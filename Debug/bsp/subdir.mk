################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../bsp/all_in_one.c" \
"../bsp/better_flash.c" \
"../bsp/board_resources.c" \
"../bsp/gps.c" \
"../bsp/imu_wifi.c" \
"../bsp/ips_sbus.c" \
"../bsp/motor_servo.c" 

COMPILED_SRCS += \
"bsp/all_in_one.src" \
"bsp/better_flash.src" \
"bsp/board_resources.src" \
"bsp/gps.src" \
"bsp/imu_wifi.src" \
"bsp/ips_sbus.src" \
"bsp/motor_servo.src" 

C_DEPS += \
"./bsp/all_in_one.d" \
"./bsp/better_flash.d" \
"./bsp/board_resources.d" \
"./bsp/gps.d" \
"./bsp/imu_wifi.d" \
"./bsp/ips_sbus.d" \
"./bsp/motor_servo.d" 

OBJS += \
"bsp/all_in_one.o" \
"bsp/better_flash.o" \
"bsp/board_resources.o" \
"bsp/gps.o" \
"bsp/imu_wifi.o" \
"bsp/ips_sbus.o" \
"bsp/motor_servo.o" 


# Each subdirectory must supply rules for building sources it contributes
"bsp/all_in_one.src":"../bsp/all_in_one.c" "bsp/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/cita/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"bsp/all_in_one.o":"bsp/all_in_one.src" "bsp/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"bsp/better_flash.src":"../bsp/better_flash.c" "bsp/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/cita/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"bsp/better_flash.o":"bsp/better_flash.src" "bsp/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"bsp/board_resources.src":"../bsp/board_resources.c" "bsp/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/cita/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"bsp/board_resources.o":"bsp/board_resources.src" "bsp/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"bsp/gps.src":"../bsp/gps.c" "bsp/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/cita/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"bsp/gps.o":"bsp/gps.src" "bsp/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"bsp/imu_wifi.src":"../bsp/imu_wifi.c" "bsp/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/cita/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"bsp/imu_wifi.o":"bsp/imu_wifi.src" "bsp/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"bsp/ips_sbus.src":"../bsp/ips_sbus.c" "bsp/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/cita/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"bsp/ips_sbus.o":"bsp/ips_sbus.src" "bsp/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"bsp/motor_servo.src":"../bsp/motor_servo.c" "bsp/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/cita/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"bsp/motor_servo.o":"bsp/motor_servo.src" "bsp/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-bsp

clean-bsp:
	-$(RM) ./bsp/all_in_one.d ./bsp/all_in_one.o ./bsp/all_in_one.src ./bsp/better_flash.d ./bsp/better_flash.o ./bsp/better_flash.src ./bsp/board_resources.d ./bsp/board_resources.o ./bsp/board_resources.src ./bsp/gps.d ./bsp/gps.o ./bsp/gps.src ./bsp/imu_wifi.d ./bsp/imu_wifi.o ./bsp/imu_wifi.src ./bsp/ips_sbus.d ./bsp/ips_sbus.o ./bsp/ips_sbus.src ./bsp/motor_servo.d ./bsp/motor_servo.o ./bsp/motor_servo.src

.PHONY: clean-bsp

