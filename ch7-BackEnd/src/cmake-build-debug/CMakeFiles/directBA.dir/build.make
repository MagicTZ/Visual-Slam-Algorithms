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
CMAKE_SOURCE_DIR = /home/magictz/Projects/slam/homework/ch7-BackEnd/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/magictz/Projects/slam/homework/ch7-BackEnd/src/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/directBA.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/directBA.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/directBA.dir/flags.make

CMakeFiles/directBA.dir/directBA.cpp.o: CMakeFiles/directBA.dir/flags.make
CMakeFiles/directBA.dir/directBA.cpp.o: ../directBA.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/magictz/Projects/slam/homework/ch7-BackEnd/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/directBA.dir/directBA.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/directBA.dir/directBA.cpp.o -c /home/magictz/Projects/slam/homework/ch7-BackEnd/src/directBA.cpp

CMakeFiles/directBA.dir/directBA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/directBA.dir/directBA.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/magictz/Projects/slam/homework/ch7-BackEnd/src/directBA.cpp > CMakeFiles/directBA.dir/directBA.cpp.i

CMakeFiles/directBA.dir/directBA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/directBA.dir/directBA.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/magictz/Projects/slam/homework/ch7-BackEnd/src/directBA.cpp -o CMakeFiles/directBA.dir/directBA.cpp.s

# Object files for target directBA
directBA_OBJECTS = \
"CMakeFiles/directBA.dir/directBA.cpp.o"

# External object files for target directBA
directBA_EXTERNAL_OBJECTS =

directBA: CMakeFiles/directBA.dir/directBA.cpp.o
directBA: CMakeFiles/directBA.dir/build.make
directBA: libbal_common.a
directBA: /usr/local/lib/libopencv_dnn.so.4.5.0
directBA: /usr/local/lib/libopencv_gapi.so.4.5.0
directBA: /usr/local/lib/libopencv_highgui.so.4.5.0
directBA: /usr/local/lib/libopencv_ml.so.4.5.0
directBA: /usr/local/lib/libopencv_objdetect.so.4.5.0
directBA: /usr/local/lib/libopencv_photo.so.4.5.0
directBA: /usr/local/lib/libopencv_stitching.so.4.5.0
directBA: /usr/local/lib/libopencv_video.so.4.5.0
directBA: /usr/local/lib/libopencv_videoio.so.4.5.0
directBA: /usr/local/lib/libpangolin.so
directBA: /usr/local/lib/libopencv_imgcodecs.so.4.5.0
directBA: /usr/local/lib/libopencv_calib3d.so.4.5.0
directBA: /usr/local/lib/libopencv_features2d.so.4.5.0
directBA: /usr/local/lib/libopencv_flann.so.4.5.0
directBA: /usr/local/lib/libopencv_imgproc.so.4.5.0
directBA: /usr/local/lib/libopencv_core.so.4.5.0
directBA: /usr/lib/x86_64-linux-gnu/libOpenGL.so
directBA: /usr/lib/x86_64-linux-gnu/libGLX.so
directBA: /usr/lib/x86_64-linux-gnu/libGLU.so
directBA: /usr/lib/x86_64-linux-gnu/libGLEW.so
directBA: /usr/lib/x86_64-linux-gnu/libEGL.so
directBA: /usr/lib/x86_64-linux-gnu/libSM.so
directBA: /usr/lib/x86_64-linux-gnu/libICE.so
directBA: /usr/lib/x86_64-linux-gnu/libX11.so
directBA: /usr/lib/x86_64-linux-gnu/libXext.so
directBA: /usr/lib/x86_64-linux-gnu/libOpenGL.so
directBA: /usr/lib/x86_64-linux-gnu/libGLX.so
directBA: /usr/lib/x86_64-linux-gnu/libGLU.so
directBA: /usr/lib/x86_64-linux-gnu/libGLEW.so
directBA: /usr/lib/x86_64-linux-gnu/libEGL.so
directBA: /usr/lib/x86_64-linux-gnu/libSM.so
directBA: /usr/lib/x86_64-linux-gnu/libICE.so
directBA: /usr/lib/x86_64-linux-gnu/libX11.so
directBA: /usr/lib/x86_64-linux-gnu/libXext.so
directBA: /usr/lib/x86_64-linux-gnu/libdc1394.so
directBA: /usr/lib/x86_64-linux-gnu/libavcodec.so
directBA: /usr/lib/x86_64-linux-gnu/libavformat.so
directBA: /usr/lib/x86_64-linux-gnu/libavutil.so
directBA: /usr/lib/x86_64-linux-gnu/libswscale.so
directBA: /usr/lib/x86_64-linux-gnu/libavdevice.so
directBA: /usr/lib/libOpenNI.so
directBA: /usr/lib/libOpenNI2.so
directBA: /usr/lib/x86_64-linux-gnu/libpng.so
directBA: /usr/lib/x86_64-linux-gnu/libz.so
directBA: /usr/lib/x86_64-linux-gnu/libjpeg.so
directBA: /usr/lib/x86_64-linux-gnu/libtiff.so
directBA: /usr/lib/x86_64-linux-gnu/libIlmImf.so
directBA: /usr/lib/x86_64-linux-gnu/liblz4.so
directBA: CMakeFiles/directBA.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/magictz/Projects/slam/homework/ch7-BackEnd/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable directBA"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/directBA.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/directBA.dir/build: directBA

.PHONY : CMakeFiles/directBA.dir/build

CMakeFiles/directBA.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/directBA.dir/cmake_clean.cmake
.PHONY : CMakeFiles/directBA.dir/clean

CMakeFiles/directBA.dir/depend:
	cd /home/magictz/Projects/slam/homework/ch7-BackEnd/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/magictz/Projects/slam/homework/ch7-BackEnd/src /home/magictz/Projects/slam/homework/ch7-BackEnd/src /home/magictz/Projects/slam/homework/ch7-BackEnd/src/cmake-build-debug /home/magictz/Projects/slam/homework/ch7-BackEnd/src/cmake-build-debug /home/magictz/Projects/slam/homework/ch7-BackEnd/src/cmake-build-debug/CMakeFiles/directBA.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/directBA.dir/depend
