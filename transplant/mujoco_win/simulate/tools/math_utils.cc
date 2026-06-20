#include "math_utils.h"

namespace wheel_leg {

double RadiansToDegrees(double radians) {
  return radians * 180.0 / kPi;
}

double DegreesToRadians(double degrees) {
  return degrees * kPi / 180.0;
}

}  // namespace wheel_leg
