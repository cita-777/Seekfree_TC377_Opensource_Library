################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# 将这些工具调用的输入和输出添加到构建变量 
C_SRCS += \
"../code/common_algorithm.c" \
"../code/menu_core.c" \
"../code/menu_display.c" \
"../code/my_math.c" \
"../code/navigation.c" \
"../code/pose.c" 

COMPILED_SRCS += \
"code/common_algorithm.src" \
"code/menu_core.src" \
"code/menu_display.src" \
"code/my_math.src" \
"code/navigation.src" \
"code/pose.src" 

C_DEPS += \
"./code/common_algorithm.d" \
"./code/menu_core.d" \
"./code/menu_display.d" \
"./code/my_math.d" \
"./code/navigation.d" \
"./code/pose.d" 

OBJS += \
"code/common_algorithm.o" \
"code/menu_core.o" \
"code/menu_display.o" \
"code/my_math.o" \
"code/navigation.o" \
"code/pose.o" 


# 每个子目录必须为构建它所贡献的源提供规则
"code/common_algorithm.src":"../code/common_algorithm.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/common_algorithm.o":"code/common_algorithm.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/menu_core.src":"../code/menu_core.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/menu_core.o":"code/menu_core.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/menu_display.src":"../code/menu_display.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/menu_display.o":"code/menu_display.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/my_math.src":"../code/my_math.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/my_math.o":"code/my_math.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/navigation.src":"../code/navigation.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/navigation.o":"code/navigation.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/pose.src":"../code/pose.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/pose.o":"code/pose.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) ./code/common_algorithm.d ./code/common_algorithm.o ./code/common_algorithm.src ./code/menu_core.d ./code/menu_core.o ./code/menu_core.src ./code/menu_display.d ./code/menu_display.o ./code/menu_display.src ./code/my_math.d ./code/my_math.o ./code/my_math.src ./code/navigation.d ./code/navigation.o ./code/navigation.src ./code/pose.d ./code/pose.o ./code/pose.src

.PHONY: clean-code

