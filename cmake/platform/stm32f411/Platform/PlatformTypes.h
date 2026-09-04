/**
 * ============================================================================
 * KLONAS Phase-1 CubeSat Flight Software
 * STM32F411 PlatformTypes.h Header
 * ============================================================================
 */

#ifndef PLATFORM_TYPES_H_
#define PLATFORM_TYPES_H_

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS 1
#endif

#include <stdint.h>
#include <inttypes.h>

#ifndef PRIu64
#define PRIu64 "llu"
#endif

#ifndef PRId64
#define PRId64 "lld"
#endif

#ifndef PRIx64
#define PRIx64 "llx"
#endif

#ifndef PRIu32
#define PRIu32 "u"
#endif

#ifndef PRId32
#define PRId32 "d"
#endif

#ifndef PRIx32
#define PRIx32 "x"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t PlatformPointerCastType;
#define PRI_PlatformPointerCastType PRIx32

#ifdef __cplusplus
}
#endif

#endif // PLATFORM_TYPES_H_
