################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# 将这些工具调用的输入和输出添加到构建变量 
C_SRCS += \
"../libraries/zf_device/lib_icm45686/inv_imu_driver.c" \
"../libraries/zf_device/lib_icm45686/inv_imu_transport.c" 

COMPILED_SRCS += \
"libraries/zf_device/lib_icm45686/inv_imu_driver.src" \
"libraries/zf_device/lib_icm45686/inv_imu_transport.src" 

C_DEPS += \
"./libraries/zf_device/lib_icm45686/inv_imu_driver.d" \
"./libraries/zf_device/lib_icm45686/inv_imu_transport.d" 

OBJS += \
"libraries/zf_device/lib_icm45686/inv_imu_driver.o" \
"libraries/zf_device/lib_icm45686/inv_imu_transport.o" 


# 每个子目录必须为构建它所贡献的源提供规则
"libraries/zf_device/lib_icm45686/inv_imu_driver.src":"../libraries/zf_device/lib_icm45686/inv_imu_driver.c" "libraries/zf_device/lib_icm45686/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/zf_device/lib_icm45686/inv_imu_driver.o":"libraries/zf_device/lib_icm45686/inv_imu_driver.src" "libraries/zf_device/lib_icm45686/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/zf_device/lib_icm45686/inv_imu_transport.src":"../libraries/zf_device/lib_icm45686/inv_imu_transport.c" "libraries/zf_device/lib_icm45686/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/zf_device/lib_icm45686/inv_imu_transport.o":"libraries/zf_device/lib_icm45686/inv_imu_transport.src" "libraries/zf_device/lib_icm45686/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-zf_device-2f-lib_icm45686

clean-libraries-2f-zf_device-2f-lib_icm45686:
	-$(RM) ./libraries/zf_device/lib_icm45686/inv_imu_driver.d ./libraries/zf_device/lib_icm45686/inv_imu_driver.o ./libraries/zf_device/lib_icm45686/inv_imu_driver.src ./libraries/zf_device/lib_icm45686/inv_imu_transport.d ./libraries/zf_device/lib_icm45686/inv_imu_transport.o ./libraries/zf_device/lib_icm45686/inv_imu_transport.src

.PHONY: clean-libraries-2f-zf_device-2f-lib_icm45686

