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
include examples/GLFW/CMakeFiles/20-map.dir/depend.make

# Include the progress variables for this target.
include examples/GLFW/CMakeFiles/20-map.dir/progress.make

# Include the compile flags for this target's objects.
include examples/GLFW/CMakeFiles/20-map.dir/flags.make

examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o: examples/GLFW/CMakeFiles/20-map.dir/flags.make
examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o: ../examples/GLFW/20-map/20-map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kronos/Downloads/chai3d-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/20-map.dir/20-map/20-map.cpp.o -c /home/kronos/Downloads/chai3d-master/examples/GLFW/20-map/20-map.cpp

examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/20-map.dir/20-map/20-map.cpp.i"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kronos/Downloads/chai3d-master/examples/GLFW/20-map/20-map.cpp > CMakeFiles/20-map.dir/20-map/20-map.cpp.i

examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/20-map.dir/20-map/20-map.cpp.s"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kronos/Downloads/chai3d-master/examples/GLFW/20-map/20-map.cpp -o CMakeFiles/20-map.dir/20-map/20-map.cpp.s

examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o.requires:

.PHONY : examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o.requires

examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o.provides: examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o.requires
	$(MAKE) -f examples/GLFW/CMakeFiles/20-map.dir/build.make examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o.provides.build
.PHONY : examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o.provides

examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o.provides.build: examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o


# Object files for target 20-map
20__map_OBJECTS = \
"CMakeFiles/20-map.dir/20-map/20-map.cpp.o"

# External object files for target 20-map
20__map_EXTERNAL_OBJECTS =

../bin/lin-x86_64/20-map: examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o
../bin/lin-x86_64/20-map: examples/GLFW/CMakeFiles/20-map.dir/build.make
../bin/lin-x86_64/20-map: libchai3d.a
../bin/lin-x86_64/20-map: /usr/lib/libGL.so
../bin/lin-x86_64/20-map: /usr/lib/libGLU.so
../bin/lin-x86_64/20-map: extras/GLFW/libglfw.a
../bin/lin-x86_64/20-map: /usr/lib/libGL.so
../bin/lin-x86_64/20-map: /usr/lib/libGLU.so
../bin/lin-x86_64/20-map: examples/GLFW/CMakeFiles/20-map.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kronos/Downloads/chai3d-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/lin-x86_64/20-map"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/20-map.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/GLFW/CMakeFiles/20-map.dir/build: ../bin/lin-x86_64/20-map

.PHONY : examples/GLFW/CMakeFiles/20-map.dir/build

examples/GLFW/CMakeFiles/20-map.dir/requires: examples/GLFW/CMakeFiles/20-map.dir/20-map/20-map.cpp.o.requires

.PHONY : examples/GLFW/CMakeFiles/20-map.dir/requires

examples/GLFW/CMakeFiles/20-map.dir/clean:
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && $(CMAKE_COMMAND) -P CMakeFiles/20-map.dir/cmake_clean.cmake
.PHONY : examples/GLFW/CMakeFiles/20-map.dir/clean

examples/GLFW/CMakeFiles/20-map.dir/depend:
	cd /home/kronos/Downloads/chai3d-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kronos/Downloads/chai3d-master /home/kronos/Downloads/chai3d-master/examples/GLFW /home/kronos/Downloads/chai3d-master/build /home/kronos/Downloads/chai3d-master/build/examples/GLFW /home/kronos/Downloads/chai3d-master/build/examples/GLFW/CMakeFiles/20-map.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/GLFW/CMakeFiles/20-map.dir/depend

