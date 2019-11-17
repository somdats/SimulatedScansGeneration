/**
 * @file   setup.h
 * @author Benjamin Russig
 *
 * Created on 10.09.2013 20:37:51
 */

#ifndef __HTEST_SETUP_H__
#define __HTEST_SETUP_H__
 
 
 
 //////
//
// Library use
//

//
// Export/import symbols
//
#ifdef DLL_EXPORT
	#define HTEST_API __declspec(dllexport)
	#define HTEST_VAL extern __declspec(dllexport)
	#define HTEST_VAL_IMPL __declspec(dllexport)
#elif defined(BUILDING_HTEST) || !defined(_WIN32)
	#define HTEST_API
	#define HTEST_VAL extern
	#define HTEST_VAL_IMPL
#else
	#if defined(HTEST_USE_STATIC)
		#define HTEST_API
	#else
		#define HTEST_API __declspec(dllimport)
	#endif
	#define HTEST_VAL extern
#endif



//////
//
// C++11 Support
//

//
// Rvalue references
//
#if (_MSC_VER >= 1600) || defined(AUTOCONF_HAVE_RVALUE_REFERENCES)
	#define HTEST_CPP11_HAS_RVALUE_REFS
#else
	namespace std
	{
		/** @brief C++11 @c std::move dummy. */
		template <typename T>
		inline const T& move(const T& a) {return a;}
	}
#endif // if (_MSC_VER >= 1600) || defined(AUTOCONF_HAVE_RVALUE_REFERENCES)

//
// Deleted members
//
#if (_MSC_VER >= 1700) || defined(AUTOCONF_DELETE_METHOD)
	#define HTEST_CPP11_HAS_DELETED_MEMBERS
#endif



//////
//
// Compiler-specific attributes, storage specifiers etc.
//

//
// Data alignment
//
#ifdef _MSC_VER
	#define HTEST_ALIGNED_UNION_BEGIN(a, name) __declspec(align(a)) union name
	#define HTEST_ALIGNED_UNION_BEGIN_ANONYMOUS(a) __declspec(align(a)) union
	#define HTEST_ALIGNED_UNION_CLOSE(a)
	#define HTEST_ALIGNED_STRUCT_BEGIN(a, name) __declspec(align(a)) struct name
	#define HTEST_ALIGNED_STRUCT_BEGIN_ANONYMOUS(a) __declspec(align(a)) struct
	#define HTEST_ALIGNED_STRUCT_CLOSE(a)
	#define HTEST_PACKED_STRUCT_BEGIN(p, name) __pragma(pack(push, p)) struct name
	#define HTEST_PACKED_STRUCT_BEGIN_ANONYMOUS(p) __pragma(pack(push, p)) struct
	#define HTEST_PACKED_STRUCT_CLOSE(p) __pragma(pack(pop))
#elif defined(__GNUC__)
	#define HTEST_ALIGNED_UNION_BEGIN(a, name) union __attribute__ ((aligned(a))) name
	#define HTEST_ALIGNED_UNION_BEGIN_ANONYMOUS(a) union __attribute__ ((aligned(a)))
	#define HTEST_ALIGNED_UNION_CLOSE(a)
	#define HTEST_ALIGNED_STRUCT_BEGIN(a, name) struct __attribute__ ((aligned(a))) name
	#define HTEST_ALIGNED_STRUCT_BEGIN_ANONYMOUS(a) struct __attribute__ ((aligned(a)))
	#define HTEST_ALIGNED_STRUCT_CLOSE(a)
	#define HTEST_PACKED_STRUCT_BEGIN(p, name) \
		struct __attribute__ ((aligned(p), __packed__)) name
	#define HTEST_PACKED_STRUCT_BEGIN_ANONYMOUS(p) \
		struct __attribute__ ((aligned(p), __packed__))
	#define HTEST_PACKED_STRUCT_CLOSE(p)
#endif // ifdef __GNUC__



//////
//
// Misc defines
//

// Define NULL-pointer if not yet defined by any other lib / include
#ifndef NULL
	#define NULL 0
#endif



#endif // ifndef __HTEST_SETUP_H__
