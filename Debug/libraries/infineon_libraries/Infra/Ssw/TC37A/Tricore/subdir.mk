################################################################################
# �Զ����ɵ��ļ�����Ҫ�༭��
################################################################################

# ����Щ���ߵ��õ�����������ӵ��������� 
C_SRCS += \
"../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.c" \
"../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.c" \
"../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.c" \
"../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.c" 

COMPILED_SRCS += \
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.src" \
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.src" \
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.src" \
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.src" 

C_DEPS += \
"./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.d" \
"./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.d" \
"./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.d" \
"./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.d" 

OBJS += \
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.o" \
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.o" \
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.o" \
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.o" 


# ÿ����Ŀ¼����Ϊ�����������׵�Դ�ṩ����
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.src":"../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.c" "libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.o":"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.src" "libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.src":"../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.c" "libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.o":"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.src" "libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.src":"../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.c" "libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.o":"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.src" "libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.src":"../libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.c" "libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.o":"libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.src" "libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-Infra-2f-Ssw-2f-TC37A-2f-Tricore

clean-libraries-2f-infineon_libraries-2f-Infra-2f-Ssw-2f-TC37A-2f-Tricore:
	-$(RM) ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.d ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.o ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Infra.src ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.d ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.o ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc0.src ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.d ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.o ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc1.src ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.d ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.o ./libraries/infineon_libraries/Infra/Ssw/TC37A/Tricore/Ifx_Ssw_Tc2.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-Infra-2f-Ssw-2f-TC37A-2f-Tricore

