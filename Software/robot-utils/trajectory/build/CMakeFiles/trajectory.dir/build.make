# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gjergji/mirmi_ws/trajectory

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gjergji/mirmi_ws/trajectory/build

# Include any dependencies generated for this target.
include CMakeFiles/trajectory.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/trajectory.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/trajectory.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/trajectory.dir/flags.make

CMakeFiles/trajectory.dir/main.cpp.o: CMakeFiles/trajectory.dir/flags.make
CMakeFiles/trajectory.dir/main.cpp.o: ../main.cpp
CMakeFiles/trajectory.dir/main.cpp.o: CMakeFiles/trajectory.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gjergji/mirmi_ws/trajectory/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/trajectory.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/trajectory.dir/main.cpp.o -MF CMakeFiles/trajectory.dir/main.cpp.o.d -o CMakeFiles/trajectory.dir/main.cpp.o -c /home/gjergji/mirmi_ws/trajectory/main.cpp

CMakeFiles/trajectory.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gjergji/mirmi_ws/trajectory/main.cpp > CMakeFiles/trajectory.dir/main.cpp.i

CMakeFiles/trajectory.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gjergji/mirmi_ws/trajectory/main.cpp -o CMakeFiles/trajectory.dir/main.cpp.s

CMakeFiles/trajectory.dir/include/trajectory.cpp.o: CMakeFiles/trajectory.dir/flags.make
CMakeFiles/trajectory.dir/include/trajectory.cpp.o: ../include/trajectory.cpp
CMakeFiles/trajectory.dir/include/trajectory.cpp.o: CMakeFiles/trajectory.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gjergji/mirmi_ws/trajectory/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/trajectory.dir/include/trajectory.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/trajectory.dir/include/trajectory.cpp.o -MF CMakeFiles/trajectory.dir/include/trajectory.cpp.o.d -o CMakeFiles/trajectory.dir/include/trajectory.cpp.o -c /home/gjergji/mirmi_ws/trajectory/include/trajectory.cpp

CMakeFiles/trajectory.dir/include/trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trajectory.dir/include/trajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gjergji/mirmi_ws/trajectory/include/trajectory.cpp > CMakeFiles/trajectory.dir/include/trajectory.cpp.i

CMakeFiles/trajectory.dir/include/trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trajectory.dir/include/trajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gjergji/mirmi_ws/trajectory/include/trajectory.cpp -o CMakeFiles/trajectory.dir/include/trajectory.cpp.s

# Object files for target trajectory
trajectory_OBJECTS = \
"CMakeFiles/trajectory.dir/main.cpp.o" \
"CMakeFiles/trajectory.dir/include/trajectory.cpp.o"

# External object files for target trajectory
trajectory_EXTERNAL_OBJECTS =

trajectory: CMakeFiles/trajectory.dir/main.cpp.o
trajectory: CMakeFiles/trajectory.dir/include/trajectory.cpp.o
trajectory: CMakeFiles/trajectory.dir/build.make
trajectory: CMakeFiles/trajectory.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gjergji/mirmi_ws/trajectory/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable trajectory"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/trajectory.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/trajectory.dir/build: trajectory
.PHONY : CMakeFiles/trajectory.dir/build

CMakeFiles/trajectory.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/trajectory.dir/cmake_clean.cmake
.PHONY : CMakeFiles/trajectory.dir/clean

CMakeFiles/trajectory.dir/depend:
	cd /home/gjergji/mirmi_ws/trajectory/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gjergji/mirmi_ws/trajectory /home/gjergji/mirmi_ws/trajectory /home/gjergji/mirmi_ws/trajectory/build /home/gjergji/mirmi_ws/trajectory/build /home/gjergji/mirmi_ws/trajectory/build/CMakeFiles/trajectory.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/trajectory.dir/depend

