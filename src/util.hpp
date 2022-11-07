#include "main.h"
#include <iomanip>

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

} // namespace utils