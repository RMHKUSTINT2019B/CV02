# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/kashunshum/Documents/Github/CV

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/kashunshum/Documents/Github/CV/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/CV.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CV.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CV.dir/flags.make

CMakeFiles/CV.dir/Source/Camera.cpp.o: CMakeFiles/CV.dir/flags.make
CMakeFiles/CV.dir/Source/Camera.cpp.o: ../Source/Camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kashunshum/Documents/Github/CV/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CV.dir/Source/Camera.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CV.dir/Source/Camera.cpp.o -c /Users/kashunshum/Documents/Github/CV/Source/Camera.cpp

CMakeFiles/CV.dir/Source/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CV.dir/Source/Camera.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kashunshum/Documents/Github/CV/Source/Camera.cpp > CMakeFiles/CV.dir/Source/Camera.cpp.i

CMakeFiles/CV.dir/Source/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CV.dir/Source/Camera.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kashunshum/Documents/Github/CV/Source/Camera.cpp -o CMakeFiles/CV.dir/Source/Camera.cpp.s

CMakeFiles/CV.dir/Source/Devices/CH341.cpp.o: CMakeFiles/CV.dir/flags.make
CMakeFiles/CV.dir/Source/Devices/CH341.cpp.o: ../Source/Devices/CH341.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kashunshum/Documents/Github/CV/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/CV.dir/Source/Devices/CH341.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CV.dir/Source/Devices/CH341.cpp.o -c /Users/kashunshum/Documents/Github/CV/Source/Devices/CH341.cpp

CMakeFiles/CV.dir/Source/Devices/CH341.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CV.dir/Source/Devices/CH341.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kashunshum/Documents/Github/CV/Source/Devices/CH341.cpp > CMakeFiles/CV.dir/Source/Devices/CH341.cpp.i

CMakeFiles/CV.dir/Source/Devices/CH341.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CV.dir/Source/Devices/CH341.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kashunshum/Documents/Github/CV/Source/Devices/CH341.cpp -o CMakeFiles/CV.dir/Source/Devices/CH341.cpp.s

CMakeFiles/CV.dir/Source/FrameProcessor.cpp.o: CMakeFiles/CV.dir/flags.make
CMakeFiles/CV.dir/Source/FrameProcessor.cpp.o: ../Source/FrameProcessor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kashunshum/Documents/Github/CV/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/CV.dir/Source/FrameProcessor.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CV.dir/Source/FrameProcessor.cpp.o -c /Users/kashunshum/Documents/Github/CV/Source/FrameProcessor.cpp

CMakeFiles/CV.dir/Source/FrameProcessor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CV.dir/Source/FrameProcessor.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kashunshum/Documents/Github/CV/Source/FrameProcessor.cpp > CMakeFiles/CV.dir/Source/FrameProcessor.cpp.i

CMakeFiles/CV.dir/Source/FrameProcessor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CV.dir/Source/FrameProcessor.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kashunshum/Documents/Github/CV/Source/FrameProcessor.cpp -o CMakeFiles/CV.dir/Source/FrameProcessor.cpp.s

CMakeFiles/CV.dir/Source/HAL/USB.cpp.o: CMakeFiles/CV.dir/flags.make
CMakeFiles/CV.dir/Source/HAL/USB.cpp.o: ../Source/HAL/USB.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kashunshum/Documents/Github/CV/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/CV.dir/Source/HAL/USB.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CV.dir/Source/HAL/USB.cpp.o -c /Users/kashunshum/Documents/Github/CV/Source/HAL/USB.cpp

CMakeFiles/CV.dir/Source/HAL/USB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CV.dir/Source/HAL/USB.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kashunshum/Documents/Github/CV/Source/HAL/USB.cpp > CMakeFiles/CV.dir/Source/HAL/USB.cpp.i

CMakeFiles/CV.dir/Source/HAL/USB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CV.dir/Source/HAL/USB.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kashunshum/Documents/Github/CV/Source/HAL/USB.cpp -o CMakeFiles/CV.dir/Source/HAL/USB.cpp.s

CMakeFiles/CV.dir/Source/VideoProcessor.cpp.o: CMakeFiles/CV.dir/flags.make
CMakeFiles/CV.dir/Source/VideoProcessor.cpp.o: ../Source/VideoProcessor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kashunshum/Documents/Github/CV/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/CV.dir/Source/VideoProcessor.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CV.dir/Source/VideoProcessor.cpp.o -c /Users/kashunshum/Documents/Github/CV/Source/VideoProcessor.cpp

CMakeFiles/CV.dir/Source/VideoProcessor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CV.dir/Source/VideoProcessor.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kashunshum/Documents/Github/CV/Source/VideoProcessor.cpp > CMakeFiles/CV.dir/Source/VideoProcessor.cpp.i

CMakeFiles/CV.dir/Source/VideoProcessor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CV.dir/Source/VideoProcessor.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kashunshum/Documents/Github/CV/Source/VideoProcessor.cpp -o CMakeFiles/CV.dir/Source/VideoProcessor.cpp.s

CMakeFiles/CV.dir/Source/main.cpp.o: CMakeFiles/CV.dir/flags.make
CMakeFiles/CV.dir/Source/main.cpp.o: ../Source/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kashunshum/Documents/Github/CV/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/CV.dir/Source/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CV.dir/Source/main.cpp.o -c /Users/kashunshum/Documents/Github/CV/Source/main.cpp

CMakeFiles/CV.dir/Source/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CV.dir/Source/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kashunshum/Documents/Github/CV/Source/main.cpp > CMakeFiles/CV.dir/Source/main.cpp.i

CMakeFiles/CV.dir/Source/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CV.dir/Source/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kashunshum/Documents/Github/CV/Source/main.cpp -o CMakeFiles/CV.dir/Source/main.cpp.s

# Object files for target CV
CV_OBJECTS = \
"CMakeFiles/CV.dir/Source/Camera.cpp.o" \
"CMakeFiles/CV.dir/Source/Devices/CH341.cpp.o" \
"CMakeFiles/CV.dir/Source/FrameProcessor.cpp.o" \
"CMakeFiles/CV.dir/Source/HAL/USB.cpp.o" \
"CMakeFiles/CV.dir/Source/VideoProcessor.cpp.o" \
"CMakeFiles/CV.dir/Source/main.cpp.o"

# External object files for target CV
CV_EXTERNAL_OBJECTS =

CV: CMakeFiles/CV.dir/Source/Camera.cpp.o
CV: CMakeFiles/CV.dir/Source/Devices/CH341.cpp.o
CV: CMakeFiles/CV.dir/Source/FrameProcessor.cpp.o
CV: CMakeFiles/CV.dir/Source/HAL/USB.cpp.o
CV: CMakeFiles/CV.dir/Source/VideoProcessor.cpp.o
CV: CMakeFiles/CV.dir/Source/main.cpp.o
CV: CMakeFiles/CV.dir/build.make
CV: /usr/local/lib/libopencv_stitching.3.4.3.dylib
CV: /usr/local/lib/libopencv_superres.3.4.3.dylib
CV: /usr/local/lib/libopencv_videostab.3.4.3.dylib
CV: /usr/local/lib/libopencv_aruco.3.4.3.dylib
CV: /usr/local/lib/libopencv_bgsegm.3.4.3.dylib
CV: /usr/local/lib/libopencv_bioinspired.3.4.3.dylib
CV: /usr/local/lib/libopencv_ccalib.3.4.3.dylib
CV: /usr/local/lib/libopencv_dnn_objdetect.3.4.3.dylib
CV: /usr/local/lib/libopencv_dpm.3.4.3.dylib
CV: /usr/local/lib/libopencv_face.3.4.3.dylib
CV: /usr/local/lib/libopencv_fuzzy.3.4.3.dylib
CV: /usr/local/lib/libopencv_hfs.3.4.3.dylib
CV: /usr/local/lib/libopencv_img_hash.3.4.3.dylib
CV: /usr/local/lib/libopencv_line_descriptor.3.4.3.dylib
CV: /usr/local/lib/libopencv_optflow.3.4.3.dylib
CV: /usr/local/lib/libopencv_reg.3.4.3.dylib
CV: /usr/local/lib/libopencv_rgbd.3.4.3.dylib
CV: /usr/local/lib/libopencv_saliency.3.4.3.dylib
CV: /usr/local/lib/libopencv_stereo.3.4.3.dylib
CV: /usr/local/lib/libopencv_structured_light.3.4.3.dylib
CV: /usr/local/lib/libopencv_surface_matching.3.4.3.dylib
CV: /usr/local/lib/libopencv_tracking.3.4.3.dylib
CV: /usr/local/lib/libopencv_xfeatures2d.3.4.3.dylib
CV: /usr/local/lib/libopencv_ximgproc.3.4.3.dylib
CV: /usr/local/lib/libopencv_xobjdetect.3.4.3.dylib
CV: /usr/local/lib/libopencv_xphoto.3.4.3.dylib
CV: /usr/local/lib/libusb-1.0.dylib
CV: /usr/local/lib/libopencv_shape.3.4.3.dylib
CV: /usr/local/lib/libopencv_photo.3.4.3.dylib
CV: /usr/local/lib/libopencv_calib3d.3.4.3.dylib
CV: /usr/local/lib/libopencv_features2d.3.4.3.dylib
CV: /usr/local/lib/libopencv_highgui.3.4.3.dylib
CV: /usr/local/lib/libopencv_videoio.3.4.3.dylib
CV: /usr/local/lib/libopencv_phase_unwrapping.3.4.3.dylib
CV: /usr/local/lib/libopencv_dnn.3.4.3.dylib
CV: /usr/local/lib/libopencv_video.3.4.3.dylib
CV: /usr/local/lib/libopencv_datasets.3.4.3.dylib
CV: /usr/local/lib/libopencv_flann.3.4.3.dylib
CV: /usr/local/lib/libopencv_ml.3.4.3.dylib
CV: /usr/local/lib/libopencv_plot.3.4.3.dylib
CV: /usr/local/lib/libopencv_imgcodecs.3.4.3.dylib
CV: /usr/local/lib/libopencv_objdetect.3.4.3.dylib
CV: /usr/local/lib/libopencv_imgproc.3.4.3.dylib
CV: /usr/local/lib/libopencv_core.3.4.3.dylib
CV: CMakeFiles/CV.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/kashunshum/Documents/Github/CV/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable CV"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CV.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CV.dir/build: CV

.PHONY : CMakeFiles/CV.dir/build

CMakeFiles/CV.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CV.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CV.dir/clean

CMakeFiles/CV.dir/depend:
	cd /Users/kashunshum/Documents/Github/CV/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/kashunshum/Documents/Github/CV /Users/kashunshum/Documents/Github/CV /Users/kashunshum/Documents/Github/CV/cmake-build-debug /Users/kashunshum/Documents/Github/CV/cmake-build-debug /Users/kashunshum/Documents/Github/CV/cmake-build-debug/CMakeFiles/CV.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CV.dir/depend

