# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/mon20/Dev/HIVE_MAP_EDITOR

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mon20/Dev/HIVE_MAP_EDITOR/build

# Include any dependencies generated for this target.
include CMakeFiles/MAP_VISUALISATOR.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MAP_VISUALISATOR.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MAP_VISUALISATOR.dir/flags.make

CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.o: CMakeFiles/MAP_VISUALISATOR.dir/flags.make
CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.o: ../software/MAP_VISUALISATOR.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mon20/Dev/HIVE_MAP_EDITOR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.o -c /home/mon20/Dev/HIVE_MAP_EDITOR/software/MAP_VISUALISATOR.cpp

CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mon20/Dev/HIVE_MAP_EDITOR/software/MAP_VISUALISATOR.cpp > CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.i

CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mon20/Dev/HIVE_MAP_EDITOR/software/MAP_VISUALISATOR.cpp -o CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.s

CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.o: CMakeFiles/MAP_VISUALISATOR.dir/flags.make
CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.o: ../src/useful.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mon20/Dev/HIVE_MAP_EDITOR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.o -c /home/mon20/Dev/HIVE_MAP_EDITOR/src/useful.cpp

CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mon20/Dev/HIVE_MAP_EDITOR/src/useful.cpp > CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.i

CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mon20/Dev/HIVE_MAP_EDITOR/src/useful.cpp -o CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.s

# Object files for target MAP_VISUALISATOR
MAP_VISUALISATOR_OBJECTS = \
"CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.o" \
"CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.o"

# External object files for target MAP_VISUALISATOR
MAP_VISUALISATOR_EXTERNAL_OBJECTS =

MAP_VISUALISATOR: CMakeFiles/MAP_VISUALISATOR.dir/software/MAP_VISUALISATOR.cpp.o
MAP_VISUALISATOR: CMakeFiles/MAP_VISUALISATOR.dir/src/useful.cpp.o
MAP_VISUALISATOR: CMakeFiles/MAP_VISUALISATOR.dir/build.make
MAP_VISUALISATOR: /usr/local/lib/libOpenXLSX.a
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
MAP_VISUALISATOR: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
MAP_VISUALISATOR: CMakeFiles/MAP_VISUALISATOR.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mon20/Dev/HIVE_MAP_EDITOR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable MAP_VISUALISATOR"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MAP_VISUALISATOR.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MAP_VISUALISATOR.dir/build: MAP_VISUALISATOR

.PHONY : CMakeFiles/MAP_VISUALISATOR.dir/build

CMakeFiles/MAP_VISUALISATOR.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MAP_VISUALISATOR.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MAP_VISUALISATOR.dir/clean

CMakeFiles/MAP_VISUALISATOR.dir/depend:
	cd /home/mon20/Dev/HIVE_MAP_EDITOR/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mon20/Dev/HIVE_MAP_EDITOR /home/mon20/Dev/HIVE_MAP_EDITOR /home/mon20/Dev/HIVE_MAP_EDITOR/build /home/mon20/Dev/HIVE_MAP_EDITOR/build /home/mon20/Dev/HIVE_MAP_EDITOR/build/CMakeFiles/MAP_VISUALISATOR.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MAP_VISUALISATOR.dir/depend
