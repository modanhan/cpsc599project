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
include examples/GLFW/CMakeFiles/31-pointcloud.dir/depend.make

# Include the progress variables for this target.
include examples/GLFW/CMakeFiles/31-pointcloud.dir/progress.make

# Include the compile flags for this target's objects.
include examples/GLFW/CMakeFiles/31-pointcloud.dir/flags.make

examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o: examples/GLFW/CMakeFiles/31-pointcloud.dir/flags.make
examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o: ../examples/GLFW/31-pointcloud/31-pointcloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kronos/Downloads/chai3d-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o -c /home/kronos/Downloads/chai3d-master/examples/GLFW/31-pointcloud/31-pointcloud.cpp

examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.i"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kronos/Downloads/chai3d-master/examples/GLFW/31-pointcloud/31-pointcloud.cpp > CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.i

examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.s"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kronos/Downloads/chai3d-master/examples/GLFW/31-pointcloud/31-pointcloud.cpp -o CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.s

examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o.requires:

.PHONY : examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o.requires

examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o.provides: examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o.requires
	$(MAKE) -f examples/GLFW/CMakeFiles/31-pointcloud.dir/build.make examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o.provides.build
.PHONY : examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o.provides

examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o.provides.build: examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o


# Object files for target 31-pointcloud
31__pointcloud_OBJECTS = \
"CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o"

# External object files for target 31-pointcloud
31__pointcloud_EXTERNAL_OBJECTS =

../bin/lin-x86_64/31-pointcloud: examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o
../bin/lin-x86_64/31-pointcloud: examples/GLFW/CMakeFiles/31-pointcloud.dir/build.make
../bin/lin-x86_64/31-pointcloud: libchai3d.a
../bin/lin-x86_64/31-pointcloud: /usr/lib/libGL.so
../bin/lin-x86_64/31-pointcloud: /usr/lib/libGLU.so
../bin/lin-x86_64/31-pointcloud: extras/GLFW/libglfw.a
../bin/lin-x86_64/31-pointcloud: /usr/lib/libGL.so
../bin/lin-x86_64/31-pointcloud: /usr/lib/libGLU.so
../bin/lin-x86_64/31-pointcloud: examples/GLFW/CMakeFiles/31-pointcloud.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kronos/Downloads/chai3d-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/lin-x86_64/31-pointcloud"
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/31-pointcloud.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/GLFW/CMakeFiles/31-pointcloud.dir/build: ../bin/lin-x86_64/31-pointcloud

.PHONY : examples/GLFW/CMakeFiles/31-pointcloud.dir/build

examples/GLFW/CMakeFiles/31-pointcloud.dir/requires: examples/GLFW/CMakeFiles/31-pointcloud.dir/31-pointcloud/31-pointcloud.cpp.o.requires

.PHONY : examples/GLFW/CMakeFiles/31-pointcloud.dir/requires

examples/GLFW/CMakeFiles/31-pointcloud.dir/clean:
	cd /home/kronos/Downloads/chai3d-master/build/examples/GLFW && $(CMAKE_COMMAND) -P CMakeFiles/31-pointcloud.dir/cmake_clean.cmake
.PHONY : examples/GLFW/CMakeFiles/31-pointcloud.dir/clean

examples/GLFW/CMakeFiles/31-pointcloud.dir/depend:
	cd /home/kronos/Downloads/chai3d-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kronos/Downloads/chai3d-master /home/kronos/Downloads/chai3d-master/examples/GLFW /home/kronos/Downloads/chai3d-master/build /home/kronos/Downloads/chai3d-master/build/examples/GLFW /home/kronos/Downloads/chai3d-master/build/examples/GLFW/CMakeFiles/31-pointcloud.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/GLFW/CMakeFiles/31-pointcloud.dir/depend

