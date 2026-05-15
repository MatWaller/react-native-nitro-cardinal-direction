#include "HybridCardinalDirection.hpp"
#import <CoreMotion/CoreMotion.h>
#import <UIKit/UIKit.h>
#include <cmath>

namespace margelo::nitro::nitrocardinaldirection {

  /*
   * Get orientation correction angle based on current device orientation.
   * Returns the angle to add to the heading to account for how the phone is rotated.
   */
  static float getOrientationCorrection() {
    UIDeviceOrientation orientation = [UIDevice currentDevice].orientation;
    switch (orientation) {
      case UIDeviceOrientationPortrait:
        return 0.0f;
      case UIDeviceOrientationLandscapeLeft:
        return 90.0f;
      case UIDeviceOrientationLandscapeRight:
        return 270.0f; // or -90.0f
      case UIDeviceOrientationPortraitUpsideDown:
        return 180.0f;
      default:
        // Unknown orientation, assume portrait
        return 0.0f;
    }
  }

 /*
     * MW - Convert degrees to cardinal direction (N, NE, E, SE, S, SW, W, NW)
     * 0 = N, 45 = NE, 90 = E, etc.
     * Each direction covers a 45 range centered on its angle (e.g. N is 337.5–22.5)
     * Adding 22.5 before dividing by 45 ensures correct rounding to nearest direction.
     * Modulo 8 wraps around from NW back to N.
 */
  static std::string degreesToCardinalStatic(float degrees) {
    static const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    int index = static_cast<int>((degrees + 22.5f) / 45.0f) % 8;
    return directions[index];
  }

  /* 
    * Instance method that calls the static method. This allows the public API to be non-static while still using the same logic.
  */
  std::string HybridCardinalDirection::degreesToCardinal(float degrees) {
    return degreesToCardinalStatic(degrees);
  }

 /*
    * Start listening to device motion updates and call the provided callback with the heading data.
    * Uses CoreMotion's device motion updates with magnetic north reference frame.
    * Applies a noise filter to only send updates when heading changes by at least 1 degree.
    */
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
      // Negate and convert to 0–360 compass bearing.
      double heading = -motion.attitude.yaw * (180.0 / M_PI);
      if (heading < 0.0) heading += 360.0;
      
      // Apply orientation correction to account for device rotation (portrait vs landscape)
      float correction = getOrientationCorrection();
      heading += correction;
      if (heading >= 360.0) heading -= 360.0;

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
