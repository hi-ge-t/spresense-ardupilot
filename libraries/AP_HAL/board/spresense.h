#pragma once

#define HAL_BOARD_NAME "SPRESENSE-M1"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE

#define HAL_STORAGE_SIZE 16384
#define HAL_STORAGE_SIZE_AVAILABLE HAL_STORAGE_SIZE

// No Copter flash budget is claimed before the first Sony linker map.
#define HAL_PROGRAM_SIZE_LIMIT_KB 0

#define HAL_OS_POSIX_IO 1
#define HAL_OS_SOCKETS 0
#define HAL_NUM_CAN_IFACES 0
#define HAL_INS_DEFAULT HAL_INS_SPRESENSE
#define HAL_GPS1_TYPE_DEFAULT 27
#define GPS_MAX_RECEIVERS 1
#define HAL_WITH_DSP 0
#define HAL_WITH_EKF_DOUBLE 0

// This bring-up target must remain impossible to arm until a later,
// separately reviewed physical-output gate explicitly removes this contract.
#define HAL_SPRESENSE_OUTPUT_DISABLED 1

#ifdef __cplusplus
#include <AP_HAL_Spresense/Semaphores.h>
#define HAL_Semaphore Spresense::Semaphore
#define HAL_BinarySemaphore Spresense::BinarySemaphore
#endif

#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 0

#define AP_SCRIPTING_ENABLED 0
#define AP_NETWORKING_ENABLED 0

// M1 keeps parameter storage on the explicitly configured microSD path, but
// does not claim a NuttX-compatible AP_Filesystem ABI or file logging yet.
// Those are separate integration gates and must not silently fall back.
#define AP_FILESYSTEM_POSIX_ENABLED 0
#define AP_FILESYSTEM_FATFS_ENABLED 0
#define AP_FILESYSTEM_LITTLEFS_ENABLED 0
#define AP_FILESYSTEM_FILE_WRITING_ENABLED 0
#define AP_FILESYSTEM_FORMAT_ENABLED 0
#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define AP_TERRAIN_AVAILABLE 0
