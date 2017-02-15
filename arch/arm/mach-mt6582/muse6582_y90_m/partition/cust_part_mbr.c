#include <partition_define.h>
#include <linux/types.h>

#if 1 /* ONE_BIN_MEMORY */
u64 MBR_START_ADDRESS_BYTE = 12288 * 1024;
#else
u64 MBR_START_ADDRESS_BYTE = 8704 * 1024;
#endif

