# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /opt/clion-2020.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2020.2.4/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/magictz/Projects/slam/homework/ch9-SlamSystem/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/BA_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/BA_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BA_test.dir/flags.make

CMakeFiles/BA_test.dir/main.cpp.o: CMakeFiles/BA_test.dir/flags.make
CMakeFiles/BA_test.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/magictz/Projects/slam/homework/ch9-SlamSystem/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BA_test.dir/main.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BA_test.dir/main.cpp.o -c /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/main.cpp

CMakeFiles/BA_test.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BA_test.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/main.cpp > CMakeFiles/BA_test.dir/main.cpp.i

CMakeFiles/BA_test.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BA_test.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/main.cpp -o CMakeFiles/BA_test.dir/main.cpp.s

CMakeFiles/BA_test.dir/MyBA.cpp.o: CMakeFiles/BA_test.dir/flags.make
CMakeFiles/BA_test.dir/MyBA.cpp.o: ../MyBA.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/magictz/Projects/slam/homework/ch9-SlamSystem/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/BA_test.dir/MyBA.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BA_test.dir/MyBA.cpp.o -c /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/MyBA.cpp

CMakeFiles/BA_test.dir/MyBA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BA_test.dir/MyBA.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/MyBA.cpp > CMakeFiles/BA_test.dir/MyBA.cpp.i

CMakeFiles/BA_test.dir/MyBA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BA_test.dir/MyBA.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/MyBA.cpp -o CMakeFiles/BA_test.dir/MyBA.cpp.s

CMakeFiles/BA_test.dir/common.cpp.o: CMakeFiles/BA_test.dir/flags.make
CMakeFiles/BA_test.dir/common.cpp.o: ../common.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/magictz/Projects/slam/homework/ch9-SlamSystem/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/BA_test.dir/common.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BA_test.dir/common.cpp.o -c /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/common.cpp

CMakeFiles/BA_test.dir/common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BA_test.dir/common.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/common.cpp > CMakeFiles/BA_test.dir/common.cpp.i

CMakeFiles/BA_test.dir/common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BA_test.dir/common.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/common.cpp -o CMakeFiles/BA_test.dir/common.cpp.s

# Object files for target BA_test
BA_test_OBJECTS = \
"CMakeFiles/BA_test.dir/main.cpp.o" \
"CMakeFiles/BA_test.dir/MyBA.cpp.o" \
"CMakeFiles/BA_test.dir/common.cpp.o"

# External object files for target BA_test
BA_test_EXTERNAL_OBJECTS =

BA_test: CMakeFiles/BA_test.dir/main.cpp.o
BA_test: CMakeFiles/BA_test.dir/MyBA.cpp.o
BA_test: CMakeFiles/BA_test.dir/common.cpp.o
BA_test: CMakeFiles/BA_test.dir/build.make
BA_test: libcommon_lib.a
BA_test: CMakeFiles/BA_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/magictz/Projects/slam/homework/ch9-SlamSystem/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable BA_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BA_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BA_test.dir/build: BA_test

.PHONY : CMakeFiles/BA_test.dir/build

CMakeFiles/BA_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BA_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BA_test.dir/clean

CMakeFiles/BA_test.dir/depend:
	cd /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/magictz/Projects/slam/homework/ch9-SlamSystem/src /home/magictz/Projects/slam/homework/ch9-SlamSystem/src /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/cmake-build-debug /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/cmake-build-debug /home/magictz/Projects/slam/homework/ch9-SlamSystem/src/cmake-build-debug/CMakeFiles/BA_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BA_test.dir/depend

