#include <android/sensor.h>
#include <android/looper.h>
#include <thread>
#include <cmath>

#include "HybridCardinalDirection.hpp"

#define SENSOR_LOOPER_ID 1

namespace margelo::nitro::nitrocardinaldirection {

  float HybridCardinalDirection::calculateAzimuth(const float* accel, const float* mag) {
    // Normalize accelerometer vector (gravity)
    float ax = accel[0], ay = accel[1], az = accel[2];
    float accelMag = std::sqrt(ax * ax + ay * ay + az * az);
    if (accelMag < 0.001f) return 0.0f; // Avoid division by zero
    ax /= accelMag;
    ay /= accelMag;
    az /= accelMag;

    // Normalize magnetometer vector
    float mx = mag[0], my = mag[1], mz = mag[2];
    float magMag = std::sqrt(mx * mx + my * my + mz * mz);
    if (magMag < 0.001f) return 0.0f; // Avoid division by zero
    mx /= magMag;
    my /= magMag;
    mz /= magMag;

    // Calculate East vector = cross product of (0,0,-1) and accel
    // East = Magnetic North x Accel
    float ex = my * az - mz * ay;
    float ey = mz * ax - mx * az;
    float ez = mx * ay - my * ax;
    float eMag = std::sqrt(ex * ex + ey * ey + ez * ez);
    if (eMag < 0.001f) return 0.0f;
    ex /= eMag;
    ey /= eMag;
    ez /= eMag;

    // Calculate North vector = accel x East
    float nx = ay * ez - az * ey;

    // Calculate azimuth using atan2
    // atan2(East component, North component) gives bearing from North
    float azimuth = std::atan2(ex, nx) * 180.0f / 3.14159265f;
    if (azimuth < 0.0f) azimuth += 360.0f;
    return azimuth;
  }

  std::string HybridCardinalDirection::degreesToCardinal(float degrees) {
    static const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    int index = static_cast<int>((degrees + 22.5) / 45.0) % 8;
    return directions[index];
  }

  bool HybridCardinalDirection::hasSignificantChange(const float* current, const float* previous, float threshold) {
    float dx = current[0] - previous[0];
    float dy = current[1] - previous[1];
    float dz = current[2] - previous[2];
    float magnitude = std::sqrt(dx * dx + dy * dy + dz * dz);
    return magnitude >= threshold;
  }

  void HybridCardinalDirection::startUpdates(const std::function<void(const SensorData&)>& callback) {
    _callback = callback;
    _isListening = true;

    _sensorManager = ASensorManager_getInstance();
    
    _accelerometer = ASensorManager_getDefaultSensor(_sensorManager, ASENSOR_TYPE_ACCELEROMETER);
    _magnet = ASensorManager_getDefaultSensor(_sensorManager, ASENSOR_TYPE_MAGNETIC_FIELD);

    std::thread([this]() {
      ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
      _sensorEventQueue = ASensorManager_createEventQueue(_sensorManager, looper, SENSOR_LOOPER_ID, nullptr, nullptr);
      
      ASensorEventQueue_enableSensor(_sensorEventQueue, _magnet);
      ASensorEventQueue_setEventRate(_sensorEventQueue, _magnet, 100000); // 100ms

      ASensorEventQueue_enableSensor(_sensorEventQueue, _accelerometer);
      ASensorEventQueue_setEventRate(_sensorEventQueue, _accelerometer, 100000); // 100ms

      ASensorEvent event;
      while (_isListening) {
        if (ALooper_pollOnce(100, nullptr, nullptr, nullptr) == SENSOR_LOOPER_ID) {
          while (ASensorEventQueue_getEvents(_sensorEventQueue, &event, 1) > 0) {
            if (_callback) {
               bool shouldUpdate = false;
               if (event.type == ASENSOR_TYPE_ACCELEROMETER) {
                 float currentAccel[3] = {event.acceleration.x, event.acceleration.y, event.acceleration.z};
                 if (hasSignificantChange(currentAccel, prevAcceleration, ACCEL_THRESHOLD)) {
                   lastAcceleration[0] = event.acceleration.x;
                   lastAcceleration[1] = event.acceleration.y;
                   lastAcceleration[2] = event.acceleration.z;
                   prevAcceleration[0] = event.acceleration.x;
                   prevAcceleration[1] = event.acceleration.y;
                   prevAcceleration[2] = event.acceleration.z;
                   shouldUpdate = true;
                 }
               } else if (event.type == ASENSOR_TYPE_MAGNETIC_FIELD) {
                 float currentMag[3] = {event.magnetic.x, event.magnetic.y, event.magnetic.z};
                 if (hasSignificantChange(currentMag, prevMag, MAG_THRESHOLD)) {
                   lastMag[0] = event.magnetic.x;
                   lastMag[1] = event.magnetic.y;
                   lastMag[2] = event.magnetic.z;
                   prevMag[0] = event.magnetic.x;
                   prevMag[1] = event.magnetic.y;
                   prevMag[2] = event.magnetic.z;
                   shouldUpdate = true;
                 }
               }

               if (shouldUpdate) {
                 float azimuth = calculateAzimuth(lastAcceleration, lastMag);
                 std::string cardinal = degreesToCardinal(azimuth);
                 SensorData data(
                   static_cast<double>(event.timestamp / 1e9),
                   static_cast<double>(azimuth),
                   cardinal
                 );

                 _callback(data);
               }
            }
          }
        }
      }

      ASensorEventQueue_disableSensor(_sensorEventQueue, _accelerometer);
      ASensorEventQueue_disableSensor(_sensorEventQueue, _magnet);

      ASensorManager_destroyEventQueue(_sensorManager, _sensorEventQueue);
      _sensorEventQueue = nullptr;
    }).detach();
  }

  void HybridCardinalDirection::stopUpdates() {
    _isListening = false;
    _callback = nullptr;
  }
} // namespace margelo::nitro::nitrocardinaldirection
