################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# 将这些工具调用的输入和输出添加到构建变量 
C_SRCS += \
"../libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.c" \
"../libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.c" 

COMPILED_SRCS += \
"libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.src" \
"libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.src" 

C_DEPS += \
"./libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.d" \
"./libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.d" 

OBJS += \
"libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.o" \
"libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.o" 


# 每个子目录必须为构建它所贡献的源提供规则
"libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.src":"../libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.c" "libraries/infineon_libraries/Configurations/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.o":"libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.src" "libraries/infineon_libraries/Configurations/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.src":"../libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.c" "libraries/infineon_libraries/Configurations/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.o":"libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.src" "libraries/infineon_libraries/Configurations/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-Configurations

clean-libraries-2f-infineon_libraries-2f-Configurations:
	-$(RM) ./libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.d ./libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.o ./libraries/infineon_libraries/Configurations/Ifx_Cfg_Ssw.src ./libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.d ./libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.o ./libraries/infineon_libraries/Configurations/Ifx_Cfg_SswBmhd.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-Configurations

