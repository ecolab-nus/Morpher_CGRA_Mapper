# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/peng/Morpher/Morpher_CGRA_Mapper

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peng/Morpher/Morpher_CGRA_Mapper/build

# Include any dependencies generated for this target.
include src/CMakeFiles/cgra_xml_mapper.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/cgra_xml_mapper.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/cgra_xml_mapper.dir/flags.make

src/CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.o: ../src/AMRRG.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/AMRRG.cpp

src/CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/AMRRG.cpp > CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/AMRRG.cpp -o CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.o: ../src/CGRA.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/CGRA.cpp

src/CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/CGRA.cpp > CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/CGRA.cpp -o CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.o: ../src/CGRA_xml_compiler.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/CGRA_xml_compiler.cpp

src/CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/CGRA_xml_compiler.cpp > CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/CGRA_xml_compiler.cpp -o CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.o: ../src/DFG.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/DFG.cpp

src/CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/DFG.cpp > CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/DFG.cpp -o CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.o: ../src/DFGEdge.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/DFGEdge.cpp

src/CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/DFGEdge.cpp > CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/DFGEdge.cpp -o CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.o: ../src/DFGNode.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/DFGNode.cpp

src/CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/DFGNode.cpp > CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/DFGNode.cpp -o CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.o: ../src/DataPath.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/DataPath.cpp

src/CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/DataPath.cpp > CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/DataPath.cpp -o CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/FU.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/FU.cpp.o: ../src/FU.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/FU.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/FU.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/FU.cpp

src/CMakeFiles/cgra_xml_mapper.dir/FU.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/FU.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/FU.cpp > CMakeFiles/cgra_xml_mapper.dir/FU.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/FU.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/FU.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/FU.cpp -o CMakeFiles/cgra_xml_mapper.dir/FU.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.o: ../src/HeuristicMapper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/HeuristicMapper.cpp

src/CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/HeuristicMapper.cpp > CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/HeuristicMapper.cpp -o CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.o: ../src/MRRGNode.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/MRRGNode.cpp

src/CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/MRRGNode.cpp > CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/MRRGNode.cpp -o CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/Module.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/Module.cpp.o: ../src/Module.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/Module.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/Module.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/Module.cpp

src/CMakeFiles/cgra_xml_mapper.dir/Module.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/Module.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/Module.cpp > CMakeFiles/cgra_xml_mapper.dir/Module.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/Module.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/Module.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/Module.cpp -o CMakeFiles/cgra_xml_mapper.dir/Module.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/PE.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/PE.cpp.o: ../src/PE.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/PE.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/PE.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/PE.cpp

src/CMakeFiles/cgra_xml_mapper.dir/PE.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/PE.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/PE.cpp > CMakeFiles/cgra_xml_mapper.dir/PE.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/PE.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/PE.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/PE.cpp -o CMakeFiles/cgra_xml_mapper.dir/PE.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.o: ../src/PathFinderMapper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/PathFinderMapper.cpp

src/CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/PathFinderMapper.cpp > CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/PathFinderMapper.cpp -o CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/Port.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/Port.cpp.o: ../src/Port.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/Port.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/Port.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/Port.cpp

src/CMakeFiles/cgra_xml_mapper.dir/Port.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/Port.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/Port.cpp > CMakeFiles/cgra_xml_mapper.dir/Port.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/Port.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/Port.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/Port.cpp -o CMakeFiles/cgra_xml_mapper.dir/Port.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.o: ../src/RegFile.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/RegFile.cpp

src/CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/RegFile.cpp > CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/RegFile.cpp -o CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.o: ../src/SimulatedAnnealingMapper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/SimulatedAnnealingMapper.cpp

src/CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/SimulatedAnnealingMapper.cpp > CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/SimulatedAnnealingMapper.cpp -o CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.o: ../src/abstract_map_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/abstract_map_main.cpp

src/CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/abstract_map_main.cpp > CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/abstract_map_main.cpp -o CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.s

src/CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.o: src/CMakeFiles/cgra_xml_mapper.dir/flags.make
src/CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.o: ../src/tinyxml2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object src/CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.o"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.o -c /home/peng/Morpher/Morpher_CGRA_Mapper/src/tinyxml2.cpp

src/CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.i"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peng/Morpher/Morpher_CGRA_Mapper/src/tinyxml2.cpp > CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.i

src/CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.s"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peng/Morpher/Morpher_CGRA_Mapper/src/tinyxml2.cpp -o CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.s

# Object files for target cgra_xml_mapper
cgra_xml_mapper_OBJECTS = \
"CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/FU.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/Module.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/PE.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/Port.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.o" \
"CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.o"

# External object files for target cgra_xml_mapper
cgra_xml_mapper_EXTERNAL_OBJECTS =

src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/AMRRG.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/CGRA.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/CGRA_xml_compiler.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/DFG.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/DFGEdge.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/DFGNode.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/DataPath.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/FU.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/HeuristicMapper.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/MRRGNode.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/Module.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/PE.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/PathFinderMapper.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/Port.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/RegFile.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/SimulatedAnnealingMapper.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/abstract_map_main.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/tinyxml2.cpp.o
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/build.make
src/cgra_xml_mapper: src/CMakeFiles/cgra_xml_mapper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/peng/Morpher/Morpher_CGRA_Mapper/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Linking CXX executable cgra_xml_mapper"
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cgra_xml_mapper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/cgra_xml_mapper.dir/build: src/cgra_xml_mapper

.PHONY : src/CMakeFiles/cgra_xml_mapper.dir/build

src/CMakeFiles/cgra_xml_mapper.dir/clean:
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build/src && $(CMAKE_COMMAND) -P CMakeFiles/cgra_xml_mapper.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/cgra_xml_mapper.dir/clean

src/CMakeFiles/cgra_xml_mapper.dir/depend:
	cd /home/peng/Morpher/Morpher_CGRA_Mapper/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peng/Morpher/Morpher_CGRA_Mapper /home/peng/Morpher/Morpher_CGRA_Mapper/src /home/peng/Morpher/Morpher_CGRA_Mapper/build /home/peng/Morpher/Morpher_CGRA_Mapper/build/src /home/peng/Morpher/Morpher_CGRA_Mapper/build/src/CMakeFiles/cgra_xml_mapper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/cgra_xml_mapper.dir/depend
