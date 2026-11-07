/* types.h - 코어 공용 타입. C2000: int 16비트, long 32비트 */
#ifndef AGV_TYPES_H
#define AGV_TYPES_H
#ifdef AGV_HOST_BUILD
#include <stdint.h>
#else
typedef unsigned int  uint16_t;
typedef int           int16_t;
typedef unsigned long uint32_t;
typedef long          int32_t;
#endif
typedef uint16_t agv_bool_t;
#define AGV_ON  1
#define AGV_OFF 0
#endif
