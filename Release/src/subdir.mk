################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CGRA.cpp \
../src/CGRA_xml_compiler.cpp \
../src/DFG.cpp \
../src/DFGEdge.cpp \
../src/DFGNode.cpp \
../src/DataPath.cpp \
../src/FU.cpp \
../src/HeuristicMapper.cpp \
../src/Module.cpp \
../src/PE.cpp \
../src/Port.cpp \
../src/RegFile.cpp \
../src/tinyxml2.cpp 

OBJS += \
./src/CGRA.o \
./src/CGRA_xml_compiler.o \
./src/DFG.o \
./src/DFGEdge.o \
./src/DFGNode.o \
./src/DataPath.o \
./src/FU.o \
./src/HeuristicMapper.o \
./src/Module.o \
./src/PE.o \
./src/Port.o \
./src/RegFile.o \
./src/tinyxml2.o 

CPP_DEPS += \
./src/CGRA.d \
./src/CGRA_xml_compiler.d \
./src/DFG.d \
./src/DFGEdge.d \
./src/DFGNode.d \
./src/DataPath.d \
./src/FU.d \
./src/HeuristicMapper.d \
./src/Module.d \
./src/PE.d \
./src/Port.d \
./src/RegFile.d \
./src/tinyxml2.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


