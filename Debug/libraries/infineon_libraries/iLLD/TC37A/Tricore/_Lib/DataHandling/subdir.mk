################################################################################
# �Զ����ɵ��ļ�����Ҫ�༭��
################################################################################

# ����Щ���ߵ��õ�����������ӵ��������� 
C_SRCS += \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.c" \
"../libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.c" 

COMPILED_SRCS += \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.src" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.src" 

C_DEPS += \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.d" \
"./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.d" 

OBJS += \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.o" \
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.o" 


# ÿ����Ŀ¼����Ϊ�����������׵�Դ�ṩ����
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.src":"../libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.c" "libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/Desktop/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=2 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.o":"libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.src" "libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-_Lib-2f-DataHandling

clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-_Lib-2f-DataHandling:
	-$(RM) ./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.asm.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_CircularBuffer.src ./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.d ./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.o ./libraries/infineon_libraries/iLLD/TC37A/Tricore/_Lib/DataHandling/Ifx_Fifo.src

.PHONY: clean-libraries-2f-infineon_libraries-2f-iLLD-2f-TC37A-2f-Tricore-2f-_Lib-2f-DataHandling

