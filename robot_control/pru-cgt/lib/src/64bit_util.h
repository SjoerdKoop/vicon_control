#include <stdint.h>
#include <limits.h>
#include <linkage.h>
#include "abi_prefix.h"

#  define _LO32(x) ((uint32_t) ((uint64_t)x))
#  define _HI32(x) ((uint32_t)(((uint64_t)x) >> 32))
#  define _MAKE64(h, l) (((uint64_t)(h) << 32) | (l))
