# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kronos/Downloads/chai3d-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kronos/Downloads/chai3d-master/build

# Include any dependencies generated for this target.
include examples/GLFW/CMakeFiles/28-voxel-basic.dir/depend.make

# Include the progress variables for this target.
include examples/GLFW/CMakeFiles/28-voxel-basic.dir/progress.make

# Include the compile flags for this target's objects.
include examples/GLFW/CMakeFiles/28-voxel-basic.dir/flags.make

examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o: examples/GLFW/CMakeFiles/28-voxel-basic.dir/flags.make
examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o: ../examples/GLFW/28-voxel-basic/28-voxel-basic.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kronos/Downloads/chai3d-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o -c /home/kronos/Downloads/chai3d-master/examples/GLFW/28-voxel-basic/28-voxel-basic.cpp

examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.i"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kronos/Downloads/chai3d-master/examples/GLFW/28-voxel-basic/28-voxel-basic.cpp > CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.i

examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.s"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kronos/Downloads/chai3d-master/examples/GLFW/28-voxel-basic/28-voxel-basic.cpp -o CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.s

examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o.requires:

.PHONY : examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o.requires

examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o.provides: examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o.requires
	$(MAKE) -f examples/GLFW/CMakeFiles/28-voxel-basic.dir/build.make examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o.provides.build
.PHONY : examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o.provides

examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o.provides.build: examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o


# Object files for target 28-voxel-basic
28__voxel__basic_OBJECTS = \
"CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o"

# External object files for target 28-voxel-basic
28__voxel__basic_EXTERNAL_OBJECTS =

../bin/lin-x86_64/28-voxel-basic: examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o
../bin/lin-x86_64/28-voxel-basic: examples/GLFW/CMakeFiles/28-voxel-basic.dir/build.make
../bin/lin-x86_64/28-voxel-basic: libchai3d.a
../bin/lin-x86_64/28-voxel-basic: /usr/lib/libGL.so
../bin/lin-x86_64/28-voxel-basic: /usr/lib/libGLU.so
../bin/lin-x86_64/28-voxel-basic: extras/GLFW/libglfw.a
../bin/lin-x86_64/28-voxel-basic: /usr/lib/libGL.so
../bin/lin-x86_64/28-voxel-basic: /usr/lib/libGLU.so
../bin/lin-x86_64/28-voxel-basic: examples/GLFW/CMakeFiles/28-voxel-basic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kronos/Downloads/chai3d-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/lin-x86_64/28-voxel-basic"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/28-voxel-basic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/GLFW/CMakeFiles/28-voxel-basic.dir/build: ../bin/lin-x86_64/28-voxel-basic

.PHONY : examples/GLFW/CMakeFiles/28-voxel-basic.dir/build

examples/GLFW/CMakeFiles/28-voxel-basic.dir/requires: examples/GLFW/CMakeFiles/28-voxel-basic.dir/28-voxel-basic/28-voxel-basic.cpp.o.requires

.PHONY : examples/GLFW/CMakeFiles/28-voxel-basic.dir/requires

examples/GLFW/CMakeFiles/28-voxel-basic.dir/clean:
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && $(CMAKE_COMMAND) -P CMakeFiles/28-voxel-basic.dir/cmake_clean.cmake
.PHONY : examples/GLFW/CMakeFiles/28-voxel-basic.dir/clean

examples/GLFW/CMakeFiles/28-voxel-basic.dir/depend:
	cd /home/kronos/Downloads/chai3d-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kronos/Downloads/chai3d-master /home/kronos/Downloads/chai3d-master/examples/GLFW /home/kronos/Downloads/chai3d-master/build /home/kronos/Downloads/chai3d-master/build/examples/GLFW /home/kronos/Downloads/chai3d-master/build/examples/GLFW/CMakeFiles/28-voxel-basic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/GLFW/CMakeFiles/28-voxel-basic.dir/depend
