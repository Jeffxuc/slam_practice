# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/jeffxu/CLion/clion-2018.3.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/jeffxu/CLion/clion-2018.3.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jeffxu/CLion/vsc++/PA2/3-geometric_train

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jeffxu/CLion/vsc++/PA2/3-geometric_train/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/3_geometric_train.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/3_geometric_train.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/3_geometric_train.dir/flags.make

CMakeFiles/3_geometric_train.dir/main.cpp.o: CMakeFiles/3_geometric_train.dir/flags.make
CMakeFiles/3_geometric_train.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jeffxu/CLion/vsc++/PA2/3-geometric_train/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/3_geometric_train.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/3_geometric_train.dir/main.cpp.o -c /home/jeffxu/CLion/vsc++/PA2/3-geometric_train/main.cpp

CMakeFiles/3_geometric_train.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3_geometric_train.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jeffxu/CLion/vsc++/PA2/3-geometric_train/main.cpp > CMakeFiles/3_geometric_train.dir/main.cpp.i

CMakeFiles/3_geometric_train.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3_geometric_train.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jeffxu/CLion/vsc++/PA2/3-geometric_train/main.cpp -o CMakeFiles/3_geometric_train.dir/main.cpp.s

# Object files for target 3_geometric_train
3_geometric_train_OBJECTS = \
"CMakeFiles/3_geometric_train.dir/main.cpp.o"

# External object files for target 3_geometric_train
3_geometric_train_EXTERNAL_OBJECTS =

3_geometric_train: CMakeFiles/3_geometric_train.dir/main.cpp.o
3_geometric_train: CMakeFiles/3_geometric_train.dir/build.make
3_geometric_train: CMakeFiles/3_geometric_train.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jeffxu/CLion/vsc++/PA2/3-geometric_train/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable 3_geometric_train"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/3_geometric_train.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/3_geometric_train.dir/build: 3_geometric_train

.PHONY : CMakeFiles/3_geometric_train.dir/build

CMakeFiles/3_geometric_train.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/3_geometric_train.dir/cmake_clean.cmake
.PHONY : CMakeFiles/3_geometric_train.dir/clean

CMakeFiles/3_geometric_train.dir/depend:
	cd /home/jeffxu/CLion/vsc++/PA2/3-geometric_train/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jeffxu/CLion/vsc++/PA2/3-geometric_train /home/jeffxu/CLion/vsc++/PA2/3-geometric_train /home/jeffxu/CLion/vsc++/PA2/3-geometric_train/cmake-build-debug /home/jeffxu/CLion/vsc++/PA2/3-geometric_train/cmake-build-debug /home/jeffxu/CLion/vsc++/PA2/3-geometric_train/cmake-build-debug/CMakeFiles/3_geometric_train.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/3_geometric_train.dir/depend

