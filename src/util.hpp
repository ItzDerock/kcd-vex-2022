#include "main.h"
#include <iomanip>

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace utils {

// map
inline double mapValue(double value, double istart, double istop, double ostart,
                       double ostop) {
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

// round to x decimal (string)
inline std::string round(double value, int decimal) {
  std::stringstream stream;
  stream << std::fixed << std::setprecision(decimal) << value;
  std::string s = stream.str();
  return s;
}

// conversions (distance)
inline double degToInch(double deg) { return (deg / 360) * (M_PI * 2.75); }
inline double inchToDeg(double inch) { return (inch / (M_PI * 2.75)) * 360; }

// conversions (angles)
inline double getRadians(double deg) { return (deg * M_PI) / 180; }

} // namespace utils
