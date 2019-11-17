
//////
//
// Includes
//

// C++ STL
#include <cmath>
#include <limits>

// Implemented header
#include "Utils.h"



//////
//
// Class implementation
//

// Constants
//

const float Constants<float>::pi = std::acos(float(-1));
const double Constants<double>::pi = std::acos(double(-1));

const float Constants<float>::eps = std::numeric_limits<float>::epsilon();
const double Constants<double>::eps = std::numeric_limits<double>::epsilon();

const float Constants<float>::inf = std::numeric_limits<float>::infinity();
const double Constants<double>::inf = std::numeric_limits<double>::infinity();

const float Constants<float>::max = std::numeric_limits<float>::max();
const double Constants<double>::max = std::numeric_limits<double>::max();
