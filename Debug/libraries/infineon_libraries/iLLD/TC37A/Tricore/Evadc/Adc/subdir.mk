################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# 将这些工具调用的输入和输出添加到构建变量 
C_SRCS += \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.c" 

COMPILED_SRCS += \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.src" 

C_DEPS += \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.d" 

OBJS += \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.o" 


# 每个子目录必须为构建它所贡献的源提供规则
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-Evadc-2f-Adc

clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-Evadc-2f-Adc:
	-$(RM) ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Evadc/Adc/IfxEvadc_Adc.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-Evadc-2f-Adc

