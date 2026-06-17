#include "HybridCardinalDirectionSpec.hpp"
#include <functional>
#include <atomic>
#include <cmath>
#include <string>
#include <memory>

#ifdef __ANDROID__
  #include <android/sensor.h>
  #include <android/looper.h>
#endif


namespace margelo::nitro::nitrocardinaldirection {
  class HybridCardinalDirection: public HybridCardinalDirectionSpec {
  public:
    HybridCardinalDirection(): HybridObject(TAG) {}
    ~HybridCardinalDirection() override = default;

  public:
    void startUpdates(const std::function<void(const SensorData&)>& callback) override;
    void stopUpdates() override;

  private:
    std::function<void(const SensorData&)> _callback;
    std::shared_ptr<std::atomic<bool>> _isListening;

#ifdef __ANDROID__
    //ASensorEventQueue* _sensorEventQueue{nullptr};
    //ASensorManager* _sensorManager{nullptr};
    //const ASensor* _accelerometer{nullptr};
    //const ASensor* _magnet{nullptr};
#endif

#ifdef __APPLE__
    void* _iosMotionManager{nullptr}; // CMMotionManager*
    float _prevHeading{0.0f};
#endif
  };
} // namespace margelo::nitro::nitrocardinaldirection
