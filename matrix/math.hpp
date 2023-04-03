#pragma once

#include <assert.h>
// #include <px4_platform_common/defines.h>
#include <sys/ioctl.h>
// #include <px4_boardconfig.h>


/****************************************************************************
 * Defines for all platforms.
 ****************************************************************************/

#define PX4_ERROR (-1)
#define PX4_OK 0

// /* Define PX4_ISFINITE */
// #ifdef __cplusplus
// static inline constexpr bool PX4_ISFINITE(float x) { return __builtin_isfinite(x); }
// static inline constexpr bool PX4_ISFINITE(double x) { return __builtin_isfinite(x); }
// #else
// #define PX4_ISFINITE(x) __builtin_isfinite(x)
// #endif /* __cplusplus */

#if defined(__PX4_NUTTX)
/****************************************************************************
 * NuttX specific defines.
 ****************************************************************************/

#define PX4_ROOTFSDIR ""
#define PX4_STORAGEDIR PX4_ROOTFSDIR "/fs/microsd"
#define _PX4_IOC(x,y) _IOC(x,y)

// mode for open with O_CREAT
#define PX4_O_MODE_777 0777
#define PX4_O_MODE_666 0666
#define PX4_O_MODE_600 0600

#elif defined(__PX4_POSIX)
/****************************************************************************
 * POSIX Specific defines
 ****************************************************************************/

// Flag is meaningless on Linux
#ifndef O_BINARY
#define O_BINARY 0
#endif

// mode for open with O_CREAT
#define PX4_O_MODE_777 (S_IRWXU | S_IRWXG | S_IRWXO)
#define PX4_O_MODE_666 (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH )
#define PX4_O_MODE_600 (S_IRUSR | S_IWUSR)

// NuttX _IOC is equivalent to Linux _IO
#define _PX4_IOC(x,y) _IO(x,y)

#define USEC_PER_TICK (1000000/PX4_TICKS_PER_SEC)
#define USEC2TICK(x) (((x)+(USEC_PER_TICK/2))/USEC_PER_TICK)

__BEGIN_DECLS
extern long PX4_TICKS_PER_SEC;
__END_DECLS

#define PX4_ROOTFSDIR CONFIG_BOARD_ROOTFSDIR

// Qurt doesn't have an SD card for storage
#ifndef __PX4_QURT
#define PX4_STORAGEDIR PX4_ROOTFSDIR
#endif

/****************************************************************************
 * Defines for POSIX and ROS
 ****************************************************************************/

#define OK 0
#define ERROR -1
#define MAX_RAND 32767

#endif // defined(__PX4_POSIX)

/* Math macro's for float literals. Do not use M_PI et al as they aren't
 * defined (neither C nor the C++ standard define math constants) */
#define M_E_F			2.71828183f
#define M_LOG2E_F		1.44269504f
#define M_LOG10E_F		0.43429448f
#define M_LN2_F			0.69314718f
#define M_LN10_F		2.30258509f
#define M_PI_F			3.14159265f
#define M_TWOPI_F		6.28318531f
#define M_PI_2_F		1.57079632f
#define M_PI_4_F		0.78539816f
#define M_3PI_4_F		2.35619449f
#define M_SQRTPI_F		1.77245385f
#define M_1_PI_F		0.31830989f
#define M_2_PI_F		0.63661977f
#define M_2_SQRTPI_F		1.12837917f
#define M_DEG_TO_RAD_F		0.0174532925f
#define M_RAD_TO_DEG_F		57.2957795f
#define M_SQRT2_F		1.41421356f
#define M_SQRT1_2_F		0.70710678f
#define M_LN2LO_F		1.90821484E-10f
#define M_LN2HI_F		0.69314718f
#define M_SQRT3_F		1.73205081f
#define M_IVLN10_F		0.43429448f	// 1 / log(10)
#define M_LOG2_E_F		0.69314718f
#define M_INVLN2_F		1.44269504f	// 1 / log(2)

/* The M_PI, as stated above, is not C standard. If you need it and
 * it isn't in your math.h file then you can use this instead. */
#define M_PI_PRECISE	3.141592653589793238462643383279502884

#define M_DEG_TO_RAD 		0.017453292519943295
#define M_RAD_TO_DEG 		57.295779513082323

#include "helper_functions.hpp"

#include "Matrix.hpp"
#include "SquareMatrix.hpp"
#include "Slice.hpp"
#include "Vector.hpp"
#include "Vector2.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"
#include "Euler.hpp"
#include "Dcm.hpp"
#include "Scalar.hpp"
#include "Quaternion.hpp"
#include "AxisAngle.hpp"
#include "LeastSquaresSolver.hpp"
#include "Dual.hpp"
#include "PseudoInverse.hpp"
#include "SparseVector.hpp"
