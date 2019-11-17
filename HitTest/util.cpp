/*
 * File:   util.cpp
 * Author: Benjamin Russig
 *
 * Created on 27.09.2013 20:11:56
 */


//////
//
// Includes
//

// C++ STL
#include <cmath>
#include <limits>
#include <ctime>
#ifndef _MSC_VER
	#include <cstdint>
#endif

// OS API
#ifdef _WIN32
	#include <windows.h>
		// What the actual f*** Micro$oft...
		#undef max
#endif

// Implemented header
#include "util.h"



//////
//
// Default namespaces
//

// Implemented namespaces
using namespace htest;
using namespace htest::util;



//////
//
// Global variables
//

// Performance counter information
struct SPerfcounterInfo_init
{
	long long frequency;
	bool available;

	SPerfcounterInfo_init()
	{
	#ifdef _WIN32
		available = QueryPerformanceFrequency((LARGE_INTEGER*)&frequency) != 0;
	#else
		perfcounter_available = false;
	#endif
	}
} perfcounter_info;



//////
//
// Function implementation
//

float htest::util::logb (float arg)
{
	return std::log(arg) / 0.693147180559945309417232121458176568075500134360255254120680009493393621969694715605863326996418687542001481f;
}

double htest::util::logb (double arg)
{
	return std::log(arg) / 0.693147180559945309417232121458176568075500134360255254120680009493393621969694715605863326996418687542001481;
}



//////
//
// Helper class implementation
//

//
// fptools
//
const float fptools<float>::EPSILON = std::numeric_limits<float>::epsilon();
const float fptools<float>::EPSILONx2 =
	std::numeric_limits<float>::epsilon()+std::numeric_limits<float>::epsilon();
const float fptools<float>::MAX_VAL = std::numeric_limits<float>::max();
const float fptools<float>::MIN_VAL = std::numeric_limits<float>::lowest();
const float fptools<float>::INFINITY_POS =  std::numeric_limits<float>::infinity();
const float fptools<float>::INFINITY_NEG = -std::numeric_limits<float>::infinity();

const double fptools<double>::EPSILON = std::numeric_limits<double>::epsilon();
const double fptools<double>::EPSILONx2 =
	std::numeric_limits<double>::epsilon()+std::numeric_limits<double>::epsilon();
const double fptools<double>::MAX_VAL = std::numeric_limits<double>::max();
const double fptools<double>::MIN_VAL = std::numeric_limits<double>::lowest();
const double fptools<double>::INFINITY_POS =  std::numeric_limits<double>::infinity();
const double fptools<double>::INFINITY_NEG = -std::numeric_limits<double>::infinity();

float fptools<float>::increment (float arg)
{
	register unsigned long tmp = *(unsigned long*)&arg;
	unsigned long inc = (tmp != 0x80000000 ? ((arg >= 0.0f) ? (tmp+1):(tmp-1)) : 1);
	return *((float*)&inc);
}

float fptools<float>::decrement (float arg)
{
	register unsigned long tmp = *(unsigned long*)&arg;
	unsigned long dec =
		((tmp<<1) != 0 ? ((arg >= 0.0f) ? (tmp-1) : (tmp+1)) : 0x80000001);
	return *((float*)&dec);
}

bool fptools<float>::equal (float val1, float val2)
{
	float difference = std::abs(val2 - val1);
    val1 = std::abs(val1);
    val2 = std::abs(val2);
	float max = (val1 > val2) ? val1 : val2;
	return difference <= max*EPSILONx2;
}

bool fptools<float>::nequal(float val1, float val2)
{
	float difference = std::abs(val2 - val1);
	val1 = std::abs(val1);
	val2 = std::abs(val2);
	float max = (val1 > val2) ? val1 : val2;
	return difference > max * EPSILONx2;
}

double fptools<double>::increment (double arg)
{
#ifdef _MSC_VER
	register unsigned __int64 tmp = *(unsigned __int64*)&arg;
	unsigned __int64 inc =
		(tmp != 0x8000000000000000 ? ((arg >= 0.0) ? (tmp+1):(tmp-1)) : 1);
#else
	register uint64_t tmp = *(uint64_t*)&arg;
	uint64_t inc = (tmp != 0x8000000000000000 ? ((arg >= 0.0) ? (tmp+1):(tmp-1)) : 1);
#endif
	return *((double*)&inc);
}

double fptools<double>::decrement (double arg)
{
#ifdef _MSC_VER
	register unsigned __int64 tmp = *(unsigned __int64*)&arg;
	unsigned __int64 dec =
		((tmp<<1) != 0 ? ((arg >= 0.0f) ? (tmp-1) : (tmp+1)) : 0x8000000000000001);
#else
	register uint64_t tmp = *(uint64_t*)&arg;
	uint64_t dec =
		((tmp<<1) != 0 ? ((arg >= 0.0f) ? (tmp-1) : (tmp+1)) : 0x8000000000000001);
#endif
	return *((double*)&dec);
}

bool fptools<double>::equal (double val1, double val2)
{
	double difference = std::abs(val2 - val1);
    val1 = std::abs(val1);
    val2 = std::abs(val2);
	double max = (val1 > val2) ? val1 : val2;
	return difference <= max*EPSILONx2;
}

bool fptools<double>::nequal(double val1, double val2)
{
	double difference = std::abs(val2 - val1);
	val1 = std::abs(val1);
	val2 = std::abs(val2);
	double max = (val1 > val2) ? val1 : val2;
	return difference > max * EPSILONx2;
}



//////
//
// Class implementation
//

//
// CStopwatch
//
CStopwatch::CStopwatch() : himpl(new long long)
{
	if (perfcounter_info.available)
		QueryPerformanceCounter((LARGE_INTEGER*)himpl);
	else
		*((long long*)himpl) = (long long)std::clock();
}

CStopwatch::~CStopwatch()
{
	delete (long long*)himpl;
}

double CStopwatch::take (void)
{
	double elapsed;

	if (perfcounter_info.available)
	{
	#ifdef _WIN32
		long long current;

		QueryPerformanceCounter((LARGE_INTEGER*)&current);
		elapsed =   double(current-(*(long long*)himpl))
		          / double(perfcounter_info.frequency);
	#endif

		return elapsed;
	}
	else
	{
		elapsed = double(std::clock() - *((long long*)himpl)) / double(CLOCKS_PER_SEC);
		return elapsed;
	}
}

double CStopwatch::reset (void)
{
	double elapsed;

	if (perfcounter_info.available)
	{
	#ifdef _WIN32
		long long current;

		QueryPerformanceCounter((LARGE_INTEGER*)&current);
		elapsed =   double(current-(*(long long*)himpl))
		          / double(perfcounter_info.frequency);
	#endif

		// Reset
		*(long long*)himpl = current;

		return elapsed;
	}
	else
	{
		std::clock_t current = std::clock();
		elapsed =  double(current - *((long long*)himpl)) / double(CLOCKS_PER_SEC);

		// Reset
		*(long long*)himpl = current;

		return elapsed;
	}
}
