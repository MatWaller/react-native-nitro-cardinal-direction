// Android-only implementation. iOS is implemented in ios/HybridCardinalDirection.mm
#ifdef __ANDROID__

#include <android/sensor.h>
#include <android/looper.h>

#include <thread>
#include <cmath>
#include <jni.h>

#include "HybridCardinalDirection.hpp"

#define SENSOR_LOOPER_ID 1

namespace margelo::nitro::nitrocardinaldirection
{

  namespace {
    constexpr float kHeadingThreshold = 1.0f;

    struct Vec3 {
      float x;
      float y;
      float z;
    };

    static bool normalize(Vec3& v) {
      float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
      if (len < 1e-6f) return false;
      v.x /= len;
      v.y /= len;
      v.z /= len;
      return true;
    }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
      return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
      };
    }

    static float circularDiff(float a, float b) {
      float diff = std::fabs(a - b);
      return std::fmin(diff, 360.0f - diff);
    }

    static bool computeBackHeadingDegrees(const Vec3& accelerometer,
                        const Vec3& magnetometer,
                                          float& outDegrees) {
      // Build earth axes in device coordinates.
      Vec3 gravity{accelerometer.x, accelerometer.y, accelerometer.z};
      Vec3 magnetic{magnetometer.x, magnetometer.y, magnetometer.z};

      if (!normalize(gravity) || !normalize(magnetic)) return false;

      Vec3 east = cross(magnetic, gravity);
      if (!normalize(east)) return false;

      Vec3 north = cross(gravity, east);
      if (!normalize(north)) return false;

      // Back-of-phone direction in device frame is negative Z.
      constexpr Vec3 back{0.0f, 0.0f, -1.0f};
      float northComponent = back.x * north.x + back.y * north.y + back.z * north.z;
      float eastComponent = back.x * east.x + back.y * east.y + back.z * east.z;

      if (std::hypot(northComponent, eastComponent) < 1e-6f) return false;

      float heading = std::atan2(eastComponent, northComponent) * (180.0f / static_cast<float>(M_PI));
      if (heading < 0.0f) heading += 360.0f;
      outDegrees = heading;
      return true;
    }

    static std::string degreesToCardinal(float degrees) {
      static const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
      int index = static_cast<int>((degrees + 22.5f) / 45.0f) % 8;
      return directions[index];
    }
  } // namespace

  void HybridCardinalDirection::startUpdates(const std::function<void(const SensorData &)> &callback)
  {
    ASensorManager *sensorManager = ASensorManager_getInstance();
    const ASensor *accelerometer = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER);
    const ASensor *magnet = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_MAGNETIC_FIELD);

    // Ensure required sensors are present before we start the listener thread.
    if (!sensorManager || !accelerometer || !magnet) {
      _isListening = std::make_shared<std::atomic<bool>>(false);
      return;
    }

    _isListening = std::make_shared<std::atomic<bool>>(true);

    auto isListening = _isListening; // shared_ptr keeps the flag alive in the thread
    auto cb = callback;

    std::thread([isListening, cb, sensorManager, accelerometer, magnet]() mutable {
      ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
      ASensorEventQueue* queue = ASensorManager_createEventQueue(sensorManager, looper, SENSOR_LOOPER_ID, nullptr, nullptr);

      if (!queue) {
        if (queue) ASensorManager_destroyEventQueue(sensorManager, queue);
        *isListening = false;
        return;
      } 
    
      // MW - Enable sensors...
      ASensorEventQueue_enableSensor(queue, accelerometer);
      ASensorEventQueue_enableSensor(queue, magnet);

      // MW - Set sensor update rates (in microseconds)
      ASensorEventQueue_setEventRate(queue, accelerometer, 1000000); // 1s
      ASensorEventQueue_setEventRate(queue, magnet, 1000000); // 1s

      ASensorEvent event;
      Vec3 latestAccelerometer{0.0f, 0.0f, 0.0f};
      Vec3 latestMagnetometer{0.0f, 0.0f, 0.0f};
      bool hasAccelerometer = false;
      bool hasMagnetometer = false;
      float prevHeading = -1.0f;

      while(isListening->load()) {
        if(ALooper_pollOnce(1000, nullptr, nullptr, nullptr) == SENSOR_LOOPER_ID) {
          while (ASensorEventQueue_getEvents(queue, &event, 1) > 0) {
            if (event.type == ASENSOR_TYPE_ACCELEROMETER) {
              latestAccelerometer = Vec3{
                static_cast<float>(event.acceleration.x),
                static_cast<float>(event.acceleration.y),
                static_cast<float>(event.acceleration.z)
              };
              hasAccelerometer = true;
            } else if (event.type == ASENSOR_TYPE_MAGNETIC_FIELD) {
              latestMagnetometer = Vec3{
                static_cast<float>(event.magnetic.x),
                static_cast<float>(event.magnetic.y),
                static_cast<float>(event.magnetic.z)
              };
              hasMagnetometer = true;
            } else {
              continue;
            }

            // Emit updates only after receiving at least one sample from both sensors.
            if (hasAccelerometer && hasMagnetometer) {
              float heading = 0.0f;
              if (!computeBackHeadingDegrees(latestAccelerometer, latestMagnetometer, heading)) {
                continue;
              }

              if (prevHeading >= 0.0f && circularDiff(heading, prevHeading) < kHeadingThreshold) {
                continue;
              }
              prevHeading = heading;

              SensorData data;
              data.degrees = heading;
              data.cardinal = degreesToCardinal(heading);
              cb(data);
            }
          }
        }
      }
      ASensorEventQueue_disableSensor(queue, accelerometer);
      ASensorEventQueue_disableSensor(queue, magnet);
      ASensorManager_destroyEventQueue(sensorManager, queue);
    }).detach();
  }

  void HybridCardinalDirection::stopUpdates()
  {
    if (_isListening)
    {
      *_isListening = false;
    }
  }
} // namespace margelo::nitro::nitrocardinaldirection

#endif // __ANDROID__
