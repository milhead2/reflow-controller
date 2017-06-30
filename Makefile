
PART=TM4C123GH6PM

TARGET=reflow

TIVA=../TivaDriver
FREERTOS=../FreeRTOSv9.0.0/FreeRTOS/Source

SIZE=arm-none-eabi-size

#Linux stuff
FLASHOPTS=-v
FLASH=lm4flash
CP=cp

#
# Echo full command lines.. Comment out for terse
VERBOSE=1
DEBUG=1

#
# Include the common make definitions.
#
include makedefs

CFLAGS += -Wno-unused-value
#CFLAGS += -save-temps

CFLAGS += -DUSB_SERIAL_OUTPUT -DMALLOC_PROVIDED 

#LIBGCC += /home/miller/bin/gcc-arm-none-eabi-5_4-2016q2/arm-none-eabi/lib/armv7e-m/fpu/libnosys.a

LDFLAGS += --stats -Map=${COMPILER}/${TARGET}.map

#
# Where to find source files that do not live in this directory.
#
VPATH+=${FREERTOS}
VPATH+=${FREERTOS}/portable/GCC/ARM_CM4F
VPATH+=${FREERTOS}/portable/MemMang

VPATH+=${TIVA}/utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=.
IPATH+=${FREERTOS}/portable/GCC/ARM_CM4F
IPATH+=${FREERTOS}/include
IPATH+=${TIVA}

#
# The default rule, which causes the FreeRTOS example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/${TARGET}.axf

#
# obscure rule to get a copy of the driver library
#
${COMPILER}/libdriver.a:
	${CP} ${TIVA}/driverlib/${COMPILER}/libdriver.a ${COMPILER}

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the FreeRTOS core.
#
${COMPILER}/${TARGET}.axf: ${COMPILER}/heap_2.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/port.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/list.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/queue.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/tasks.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/timers.o

${COMPILER}/${TARGET}.axf: ${COMPILER}/uartstdio.o



#
# Rules for building the application
#
${COMPILER}/${TARGET}.axf: ${COMPILER}/main.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/syscalls.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/display.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/assert.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/startup_${COMPILER}.o

#
# Tiva driver library
#
${COMPILER}/${TARGET}.axf: ${COMPILER}/libdriver.a



#
# load map and device memory description
#
${COMPILER}/${TARGET}.axf: ti_tm4c123g.ld

SCATTERgcc_${TARGET}=ti_tm4c123g.ld
ENTRY_${TARGET}=ResetISR
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RB1

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

#
# A rule to flash and restarrt the program
#
flash: ${COMPILER}/${TARGET}.axf
	${SIZE} ${COMPILER}/${TARGET}.axf
	${FLASH} ${FLASHOPTS} ${COMPILER}/${TARGET}.bin
