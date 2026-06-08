// Android-only implementation. iOS is implemented in ios/HybridCardinalDirection.mm
#ifdef __ANDROID__

#include <android/sensor.h>
#include <android/looper.h>

#include <thread>
#include <cmath>
#include <jni.h>

// Store JavaVM pointer globally
JavaVM* g_JavaVM = nullptr;

#include "HybridCardinalDirection.hpp"

#define SENSOR_LOOPER_ID 1

namespace margelo::nitro::nitrocardinaldirection {

  /*
   * MW - Remap current sensor coordinates back to portrait orientation.
   * This makes heading calculations consistent regardless of device rotation.
   */
  static void remapToPortrait(int displayRotation, const float* in, float* out) {
    switch (displayRotation) {
      case 1: // Surface.ROTATION_90
        out[0] = in[1];
        out[1] = -in[0];
        out[2] = in[2];
        break;
      case 2: // Surface.ROTATION_180
        out[0] = -in[0];
        out[1] = -in[1];
        out[2] = in[2];
        break;
      case 3: // Surface.ROTATION_270
        out[0] = -in[1];
        out[1] = in[0];
        out[2] = in[2];
        break;
      default: // Surface.ROTATION_0 or unknown
        out[0] = in[0];
        out[1] = in[1];
        out[2] = in[2];
        break;
    }
  }

  float HybridCardinalDirection::calculateAzimuth(const float* accel, const float* mag, int displayRotation) {
    float rotatedAccel[3];
    float rotatedMag[3];
    remapToPortrait(displayRotation, accel, rotatedAccel);
    remapToPortrait(displayRotation, mag, rotatedMag);

    // Normalize accelerometer vector (gravity)
    float ax = rotatedAccel[0], ay = rotatedAccel[1], az = rotatedAccel[2];
    float accelMag = std::sqrt(ax * ax + ay * ay + az * az);
    if (accelMag < 0.001f) return 0.0f; // Avoid division by zero
    ax /= accelMag;
    ay /= accelMag;
    az /= accelMag;

    // Normalize magnetometer vector
    float mx = rotatedMag[0], my = rotatedMag[1], mz = rotatedMag[2];
    float magMag = std::sqrt(mx * mx + my * my + mz * mz);
    if (magMag < 0.001f) return 0.0f; // Avoid division by zero
    mx /= magMag;
    my /= magMag;
    mz /= magMag;

    // MW - Calculate East vector = cross product of magnetic field and gravity
    float ex = my * az - mz * ay;
    float ey = mz * ax - mx * az;
    float ez = mx * ay - my * ax;
    float eMag = std::sqrt(ex * ex + ey * ey + ez * ez);
    if (eMag < 0.001f) return 0.0f;
    ex /= eMag;
    ey /= eMag;
    ez /= eMag;

    // Calculate North vector = gravity x East
    float nx = ay * ez - az * ey;
    float ny = az * ex - ax * ez;
    float nz = ax * ey - ay * ex;

    // MW - Calculate azimuth using the device top-edge (Y axis) projection.
    // atan2(-East_Y, North_Y) returns heading from North, clockwise.
    float azimuth = std::atan2(-ey, ny) * 180.0f / 3.14159265f;
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
    
    _displayRotation = 0;
    _prevHeading = -1.0f;

    std::thread([this]() {
      ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
      _sensorEventQueue = ASensorManager_createEventQueue(_sensorManager, looper, SENSOR_LOOPER_ID, nullptr, nullptr);
      
      ASensorEventQueue_enableSensor(_sensorEventQueue, _magnet);
      ASensorEventQueue_setEventRate(_sensorEventQueue, _magnet, 100000); // 100ms

      ASensorEventQueue_enableSensor(_sensorEventQueue, _accelerometer);
      ASensorEventQueue_setEventRate(_sensorEventQueue, _accelerometer, 100000); // 100ms

      ASensorEvent event;


      JNIEnv* env = nullptr;

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
                 // Fetch display rotation from OrientationHelper via JNI

                 int rotation = 0;
                 if (g_JavaVM) {
                   jint getEnvStat = g_JavaVM->GetEnv((void**)&env, JNI_VERSION_1_6);
                   bool didAttach = false;
                   if (getEnvStat == JNI_EDETACHED) {
                     if (g_JavaVM->AttachCurrentThread(&env, nullptr) == 0) {
                       didAttach = true;
                     }
                   }
                   if (env) {
                     jclass helperClass = env->FindClass("com/margelo/nitro/nitrocardinaldirection/OrientationHelper");
                     if (helperClass) {
                       jmethodID getRotation = env->GetStaticMethodID(helperClass, "getDisplayRotation", "()I");
                       if (getRotation) {
                         rotation = env->CallStaticIntMethod(helperClass, getRotation);
                       }
                       env->DeleteLocalRef(helperClass);
                     }
                   }
                   if (didAttach) {
                     g_JavaVM->DetachCurrentThread();
                   }
                 }
                 _displayRotation = rotation;

                 float azimuth = calculateAzimuth(lastAcceleration, lastMag, _displayRotation);
                 if (_prevHeading >= 0.0f) {
                   float diff = std::fabs(azimuth - _prevHeading);
                   if (diff > 180.0f) diff = 360.0f - diff;
                   if (diff < HEADING_THRESHOLD) {
                     continue;
                   }
                 }
                 _prevHeading = azimuth;
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

#endif // __ANDROID__
