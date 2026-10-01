/* MSVC shims for libsbc */
#include <stdint.h>
#include <stddef.h>
#ifdef _MSC_VER
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#define __LITTLE_ENDIAN 1234
#define __BYTE_ORDER 1234
#endif
