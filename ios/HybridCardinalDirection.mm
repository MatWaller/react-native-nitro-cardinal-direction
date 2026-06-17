#include "HybridCardinalDirection.hpp"
#import <CoreMotion/CoreMotion.h>
#include <cmath>

namespace margelo::nitro::nitrocardinaldirection {

  namespace {
    constexpr float kHeadingThreshold = 1.0f;

    static float circularDiff(float a, float b) {
      float diff = std::fabs(a - b);
      return std::fmin(diff, 360.0f - diff);
    }

    static std::string degreesToCardinal(float degrees) {
      static const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
      int index = static_cast<int>((degrees + 22.5f) / 45.0f) % 8;
      return directions[index];
    }

    static bool computeBackHeadingDegrees(const CMRotationMatrix& rotationMatrix, float& outHeading) {
      // rotationMatrix transforms from reference frame to device frame.
      // The back-of-phone direction in device coordinates is negative Z.
      double north = -rotationMatrix.m31;
      double west = -rotationMatrix.m32;
      double east = -west;

      if (std::hypot(north, east) < 1e-6) {
        return false;
      }

      double heading = std::atan2(east, north) * (180.0 / M_PI);
      if (heading < 0.0) heading += 360.0;
      outHeading = static_cast<float>(heading);
      return true;
    }
  } // namespace

 /*
    * Start listening to device motion updates and call the provided callback with the heading data.
    * Uses CoreMotion's device motion updates with magnetic north reference frame and computes
    * heading from the back-of-phone vector so orientation changes remain normalized.
    * Applies a noise filter to only send updates when heading changes by at least 1 degree.
    */
  void HybridCardinalDirection::startUpdates(const std::function<void(const SensorData&)>& callback) {
    _isListening = std::make_shared<std::atomic<bool>>(true);
    _callback = callback;
    _prevHeading = -1.0f;

    CMMotionManager* manager = [[CMMotionManager alloc] init];
    auto isListening = _isListening;

    // Ensure required motion/magnetic capabilities are available before starting updates.
    if (!manager.deviceMotionAvailable ||
        (CMMotionManager.availableAttitudeReferenceFrames & CMAttitudeReferenceFrameXMagneticNorthZVertical) == 0) {
      *isListening = false;
      _callback = nullptr;
      return;
    }

    // ARC bridge: retain the ObjC object in a C++ void* member
    _iosMotionManager = (__bridge_retained void*)manager;

    manager.deviceMotionUpdateInterval = 0.1; // 100ms, matches Android rate

    NSOperationQueue* queue = [[NSOperationQueue alloc] init];

    // Capture by pointer/shared state so the block can observe stopUpdates() calls.
    auto* callbackPtr = &_callback;
    auto* prevHeadingPtr = &_prevHeading;

    [manager startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXMagneticNorthZVertical
                                                toQueue:queue
                                            withHandler:^(CMDeviceMotion* motion, NSError* error) {
      if (!isListening->load() || !*callbackPtr || error || motion == nil) return;

      float heading = 0.0f;
      if (!computeBackHeadingDegrees(motion.attitude.rotationMatrix, heading)) {
        return;
      }

      // Skip update if change is less than 1 degree (noise filter)
      if (*prevHeadingPtr >= 0.0f && circularDiff(heading, *prevHeadingPtr) < kHeadingThreshold) {
        return;
      }
      *prevHeadingPtr = heading;

      SensorData data;
      data.degrees = heading;
      data.cardinal = degreesToCardinal(heading);
      (*callbackPtr)(data);
    }];
  }

  void HybridCardinalDirection::stopUpdates() {
    if (_isListening) {
      *_isListening = false;
    }
    _callback = nullptr;

    if (_iosMotionManager) {
      // Transfer ownership back to ARC so the object is properly released
      CMMotionManager* manager = (__bridge_transfer CMMotionManager*)_iosMotionManager;
      [manager stopDeviceMotionUpdates];
      _iosMotionManager = nullptr;
    }
  }

} // namespace margelo::nitro::nitrocardinaldirection
