/**
 * @file   util.h
 * @author Benjamin Russig
 *
 * Created on 10.09.2013 22:51:37
 */

#ifndef __HTEST_UTIL_H__
#define __HTEST_UTIL_H__


// Library setup
#include "_config.h"



// Open library namespace
namespace htest {

// Open module namespace
namespace util {



//////
//
// Helper makros
//

/** @brief Undocumented. Meant for internal use by the library. */
#define HTEST_CRASH_AND_BURN (*((int*)NULL) = 0)



//////
//
// Functions
//

/**
 * @brief Returns the binary logarithm of the argument (single precision).
 *
 * @return the binary logarithm.
 *
 * @param arg
 *		The argument.
 */
HTEST_API float logb (float arg);

/**
 * @brief Returns the binary logarithm of the argument (double precision).
 *
 * @return the binary logarithm.
 *
 * @param arg
 *		The argument.
 */
HTEST_API double logb (double arg);



//////
//
// Helper classes
//

/**
 * @brief
 *		A simple RAII structure wrapping a C-style array pointer to prevent memory
 *		leaks.
 */
template <class T> struct SArrayPointerContainer
{
	/** @brief The wrapped C-style array pointer*/
	T *ptr;

	/** @brief The default constructor. Initializes the wrapped pointer to NULL. */
	SArrayPointerContainer() : ptr(NULL) {};

	/** @brief Constructs container with an initial wrapped pointer. */
	SArrayPointerContainer(T* init) : ptr(init) {};

	/** @brief The destructor. */
	~SArrayPointerContainer()
	{
		if (ptr != NULL)
			delete [] ptr;
	}

	/** @brief Assigns a new pointer for wrapping. */
	inline T* operator = (T* other)
	{
		if (ptr != NULL)
			delete [] ptr;
		return ptr = other;
	}

	/** @brief Explicitely frees the memory behind the wrapped pointer. */
	inline void free(void)
	{
		if (ptr != NULL)
		{
			delete [] ptr;
			ptr = NULL;
		}
	}
};

// Only include this utility class if the including module actually uses an std::map
// ToDo: find out and add more STL map include guards from other compilers and versions.
#if defined(_MAP_) || defined(_GLIBCXX_MAP)
/**
 * @brief A utility class for statically initializing an @c std::map .
 *
 * Example usage:<br />
 * @code std::map old_japanese_numbers = htest::util::init_map<int, char*>(1, "hi")(2,
 * "fu")(3, "mi"); @endcode
 */
template <class key_type, class value_type>
class init_map
{
	/** @brief The internal map used for compiling the initialization. */
	std::map<key_type, value_type> m;

public:

	/** @brief Constructor. Creates the first map entry. */
	inline init_map(const key_type &key, const value_type &value)
	{
		m[key] = value;
	}

	/**
	 * @brief
	 *		Calling operator. Creates subsequent map entries in a syntactically nice way.
	 */
	inline init_map<key_type, value_type>& operator() (const key_type &key,
	                                                   const value_type &value)
	{
		m[key] = value;
		return *this;
	}

	/** @brief Conversion operator to @c std::map . */
	inline operator std::map<key_type, value_type> (void)
    {
        return m;
    }
};
#endif


/** @brief Static provider for floating point related constants and utility functions. */
template<class flt_type> struct HTEST_API fptools;
/**
 * @brief
 *		Static provider for single-precision floating point related constants and utility
 *		functions.
 */
template<> struct HTEST_API fptools<float>
{
	static const float EPSILON;
	static const float EPSILONx2;
	static const float MAX_VAL;
	static const float MIN_VAL;
	static const float INFINITY_POS;
	static const float INFINITY_NEG;

	/**
	 * @brief
	 *		Returns the increment of a float value, i.e. the smallest float-representable
	 *		number @a r so that @a r > @a arg .
	 *
	 * @param arg
	 *		The float value to increment.
	 */
	static float increment (float arg);

	/**
	 * @brief
	 *		Returns the decrement of a float value, i.e. the biggest float-representable
	 *		number @a r so that @a r < @a arg .
	 *
	 * @param arg
	 *		The float value to decrement.
	 */
	static float decrement (float arg);

	/**
	 * @brief
	 *		In rounding error-prone floating point calculation scenarios, determines if
	 *		two float values are meant to represent the same real number by comparing
	 *		their difference to the minimal floating point error of the greater of both
	 *		values.
	 *
	 * @param val1
	 *		The float value to check against @c val2
	 * @param val2
	 *		The float value to check against @c val1
	 */
	static bool equal (float val1, float val2);

	/**
	* @brief
	*		In rounding error-prone floating point calculation scenarios, determines if
	*		two float values are meant to represent different real numbers by comparing
	*		their difference to the minimal floating point error of the greater of both
	*		values.
	*
	* @param val1
	*		The float value to check against @c val2
	* @param val2
	*		The float value to check against @c val1
	*/
	static bool nequal(float val1, float val2);
};
/**
 * @brief
 *		Static provider for double-precision floating point related constants and utility
 *		functions.
 */
template<> struct HTEST_API fptools<double>
{
	static const double EPSILON;
	static const double EPSILONx2;
	static const double MAX_VAL;
	static const double MIN_VAL;
	static const double INFINITY_POS;
	static const double INFINITY_NEG;

	/**
	 * @brief
	 *		Returns the increment of a double value, i.e. the smallest
	 *		double-representable number @a r so that @a r > @a arg .
	 *
	 * @param arg
	 *		The double value to increment.
	 */
	static double increment (double arg);

	/**
	 * @brief
	 *		Returns the decrement of a double value, i.e. the biggest
	 *		double-representable number @a r so that @a r < @a arg .
	 *
	 * @param arg
	 *		The double value to decrement.
	 */
	static double decrement (double arg);

	/**
	 * @brief
	 *		In rounding error-prone floating point calculation scenarios, determines if
	 *		two double values are meant to represent the same real number by comparing
	 *		their difference to the minimal floating point error of the greater of both
	 *		values.
	 *
	 * @param val1
	 *		The double value to check against @c val2
	 * @param val2
	 *		The double value to check against @c val1
	 */
	static bool equal (double val1, double val2);

	/**
	* @brief
	*		In rounding error-prone floating point calculation scenarios, determines if
	*		two float values are meant to represent different real numbers by comparing
	*		their difference to the minimal floating point error of the greater of both
	*		values.
	*
	* @param val1
	*		The float value to check against @c val2
	* @param val2
	*		The float value to check against @c val1
	*/
	static bool nequal(double val1, double val2);
};



//////
//
// Classes
//

/** @brief A simple stopwatch class for easy benchmarking. */
class CStopwatch
{
private:

	////
	// Private data members

	/** @brief Implementation handle. */
	void* himpl;


public:

	////
	// Object construction / destruction

	/** @brief Default constructor. Also starts the stopwatch. */
	CStopwatch();

	/** @brief The destructor. */
	~CStopwatch();


	////
	// Methods

	/** @brief Takes the current time in seconds. */
	double take (void);

	/**
	 * @brief
	 *		Takes the current time in seconds, and resets the stopwatch to continue
	 *		counting from zero.
	 */
	double reset (void);
};



// Close module namespace
}

// Close library namespace
}


#endif // ifndef __HTEST_UTIL_H__
