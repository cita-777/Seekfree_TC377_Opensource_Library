################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# 将这些工具调用的输入和输出添加到构建变量 
C_SRCS += \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.c" 

COMPILED_SRCS += \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.src" 

C_DEPS += \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.d" 

OBJS += \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.o" 


# 每个子目录必须为构建它所贡献的源提供规则
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-Gtm-2f-Std

clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-Gtm-2f-Std:
	-$(RM) ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Atom.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Cmu.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dpll.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Dtm.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Psm.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Spe.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tbu.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tim.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/Gtm/Std/IfxGtm_Tom.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-Gtm-2f-Std

