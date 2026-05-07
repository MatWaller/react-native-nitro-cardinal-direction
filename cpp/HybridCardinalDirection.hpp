#include "HybridCardinalDirectionSpec.hpp"
#include <functional>
#include <atomic>
#include <android/sensor.h>
#include <cmath>

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
    std::atomic<bool> _isListening{false};
    float calculateAzimuth(const float* accel, const float* mag);
    std::string degreesToCardinal(float degrees);
    bool hasSignificantChange(const float* current, const float* previous, float threshold);
    ASensorEventQueue* _sensorEventQueue{nullptr};
    ASensorManager* _sensorManager{nullptr};
    const ASensor* _accelerometer{nullptr};
    const ASensor* _magnet{nullptr};
    float lastAcceleration[3]{0, 0, 0};
    float lastMag[3]{0, 0, 0};
    float prevAcceleration[3]{0, 0, 0};
    float prevMag[3]{0, 0, 0};
    static constexpr float ACCEL_THRESHOLD = 0.5f;
    static constexpr float MAG_THRESHOLD = 5.0f;
  };
} // namespace margelo::nitro::nitrocardinaldirection
