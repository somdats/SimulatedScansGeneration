
//////
//
// Includes
//

// Local includes
#include "Ray.h"

// Implemented header
#include "IScannableObject.h"



//////
//
// Interface pre-implementation
//

// IScannableObject
//

template <class flt_type>
IScannableObject<flt_type>::~IScannableObject() {}



//////
//
// Explicit template instantiations
//

// Only floating point variants are intended
template APISPEC IScannableObject<float>;
template APISPEC IScannableObject<double>;
