#####
# PlatformTypes.fpp for STM32F411 (ARM Cortex-M4F 32-bit Architecture)
#####

@ The unsigned type of larger sizes internal to the software
type PlatformSizeType = U32

@ The signed type of larger sizes internal to the software
type PlatformSignedSizeType = I32

@ The type of smaller indices internal to the software
type PlatformIndexType = I16

@ The type of arguments to assert functions
type PlatformAssertArgType = I32

@ The type of task priorities used
type PlatformTaskPriorityType = U8

@ The type of task identifiers
type PlatformTaskIdType = I32

@ The type of queue priorities used
type PlatformQueuePriorityType = U8
