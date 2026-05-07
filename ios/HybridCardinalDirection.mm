#include "HybridCardinalDirection.hpp"
#import <CoreMotion/CoreMotion.h>
#include <cmath>

namespace margelo::nitro::nitrocardinaldirection {

  static std::string degreesToCardinalStatic(float degrees) {
    static const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    int index = static_cast<int>((degrees + 22.5f) / 45.0f) % 8;
    return directions[index];
  }

  // Delegate degreesToCardinal to the shared static helper
  std::string HybridCardinalDirection::degreesToCardinal(float degrees) {
    return degreesToCardinalStatic(degrees);
  }

  void HybridCardinalDirection::startUpdates(const std::function<void(const SensorData&)>& callback) {
    _callback = callback;
    _isListening = true;
    _prevHeading = -1.0f; // Force first update through

    CMMotionManager* manager = [[CMMotionManager alloc] init];
    // ARC bridge: retain the ObjC object in a C++ void* member
    _iosMotionManager = (__bridge_retained void*)manager;

    if (!manager.deviceMotionAvailable) {
      return;
    }

    manager.deviceMotionUpdateInterval = 0.1; // 100ms, matches Android rate

    NSOperationQueue* queue = [[NSOperationQueue alloc] init];

    // Capture by pointer so the block can observe live state changes from stopUpdates()
    auto* callbackPtr = &_callback;
    auto* listeningPtr = &_isListening;
    auto* prevHeadingPtr = &_prevHeading;

    [manager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXMagneticNorthZVertical
                                                toQueue:queue
                                            withHandler:^(CMDeviceMotion* motion, NSError* error) {
      if (!listeningPtr->load() || !*callbackPtr || error) return;

      // yaw is rotation around the vertical axis (Z), relative to magnetic north.
      // Negate and convert to 0–360° compass bearing.
      double heading = -motion.attitude.yaw * (180.0 / M_PI);
      if (heading < 0.0) heading += 360.0;

      // Skip update if change is less than 1 degree (noise filter)
      float headingF = static_cast<float>(heading);
      float diff = std::abs(headingF - *prevHeadingPtr);
      if (diff < 360.0f && diff < 1.0f) return; // diff == 360 on first run (prevHeading == -1)
      *prevHeadingPtr = headingF;

      std::string cardinal = degreesToCardinalStatic(headingF);
      SensorData data(motion.timestamp, heading, cardinal);
      (*callbackPtr)(data);
    }];
  }

  void HybridCardinalDirection::stopUpdates() {
    _isListening = false;
    _callback = nullptr;

    if (_iosMotionManager) {
      // Transfer ownership back to ARC so the object is properly released
      CMMotionManager* manager = (__bridge_transfer CMMotionManager*)_iosMotionManager;
      [manager stopDeviceMotionUpdates];
      _iosMotionManager = nullptr;
    }
  }

} // namespace margelo::nitro::nitrocardinaldirection
