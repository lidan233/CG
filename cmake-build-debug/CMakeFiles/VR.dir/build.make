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
CMAKE_COMMAND = /home/lidan/Desktop/clion-2020.3.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/lidan/Desktop/clion-2020.3.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lidan/Desktop/CG/CG

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lidan/Desktop/CG/CG/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/VR.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/VR.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/VR.dir/flags.make

CMakeFiles/VR.dir/main.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/VR.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/main.cpp.o -c /home/lidan/Desktop/CG/CG/main.cpp

CMakeFiles/VR.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/main.cpp > CMakeFiles/VR.dir/main.cpp.i

CMakeFiles/VR.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/main.cpp -o CMakeFiles/VR.dir/main.cpp.s

CMakeFiles/VR.dir/external/glad/src/glad.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/external/glad/src/glad.cpp.o: ../external/glad/src/glad.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/VR.dir/external/glad/src/glad.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/external/glad/src/glad.cpp.o -c /home/lidan/Desktop/CG/CG/external/glad/src/glad.cpp

CMakeFiles/VR.dir/external/glad/src/glad.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/external/glad/src/glad.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/external/glad/src/glad.cpp > CMakeFiles/VR.dir/external/glad/src/glad.cpp.i

CMakeFiles/VR.dir/external/glad/src/glad.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/external/glad/src/glad.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/external/glad/src/glad.cpp -o CMakeFiles/VR.dir/external/glad/src/glad.cpp.s

CMakeFiles/VR.dir/math/MatrixCalculation.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/math/MatrixCalculation.cpp.o: ../math/MatrixCalculation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/VR.dir/math/MatrixCalculation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/math/MatrixCalculation.cpp.o -c /home/lidan/Desktop/CG/CG/math/MatrixCalculation.cpp

CMakeFiles/VR.dir/math/MatrixCalculation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/math/MatrixCalculation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/math/MatrixCalculation.cpp > CMakeFiles/VR.dir/math/MatrixCalculation.cpp.i

CMakeFiles/VR.dir/math/MatrixCalculation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/math/MatrixCalculation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/math/MatrixCalculation.cpp -o CMakeFiles/VR.dir/math/MatrixCalculation.cpp.s

CMakeFiles/VR.dir/geometry/Rectangle.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/geometry/Rectangle.cpp.o: ../geometry/Rectangle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/VR.dir/geometry/Rectangle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/geometry/Rectangle.cpp.o -c /home/lidan/Desktop/CG/CG/geometry/Rectangle.cpp

CMakeFiles/VR.dir/geometry/Rectangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/geometry/Rectangle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/geometry/Rectangle.cpp > CMakeFiles/VR.dir/geometry/Rectangle.cpp.i

CMakeFiles/VR.dir/geometry/Rectangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/geometry/Rectangle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/geometry/Rectangle.cpp -o CMakeFiles/VR.dir/geometry/Rectangle.cpp.s

CMakeFiles/VR.dir/geometry/Size.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/geometry/Size.cpp.o: ../geometry/Size.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/VR.dir/geometry/Size.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/geometry/Size.cpp.o -c /home/lidan/Desktop/CG/CG/geometry/Size.cpp

CMakeFiles/VR.dir/geometry/Size.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/geometry/Size.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/geometry/Size.cpp > CMakeFiles/VR.dir/geometry/Size.cpp.i

CMakeFiles/VR.dir/geometry/Size.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/geometry/Size.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/geometry/Size.cpp -o CMakeFiles/VR.dir/geometry/Size.cpp.s

CMakeFiles/VR.dir/geometry/SpacePartition.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/geometry/SpacePartition.cpp.o: ../geometry/SpacePartition.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/VR.dir/geometry/SpacePartition.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/geometry/SpacePartition.cpp.o -c /home/lidan/Desktop/CG/CG/geometry/SpacePartition.cpp

CMakeFiles/VR.dir/geometry/SpacePartition.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/geometry/SpacePartition.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/geometry/SpacePartition.cpp > CMakeFiles/VR.dir/geometry/SpacePartition.cpp.i

CMakeFiles/VR.dir/geometry/SpacePartition.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/geometry/SpacePartition.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/geometry/SpacePartition.cpp -o CMakeFiles/VR.dir/geometry/SpacePartition.cpp.s

CMakeFiles/VR.dir/geometry/Vertex.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/geometry/Vertex.cpp.o: ../geometry/Vertex.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/VR.dir/geometry/Vertex.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/geometry/Vertex.cpp.o -c /home/lidan/Desktop/CG/CG/geometry/Vertex.cpp

CMakeFiles/VR.dir/geometry/Vertex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/geometry/Vertex.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/geometry/Vertex.cpp > CMakeFiles/VR.dir/geometry/Vertex.cpp.i

CMakeFiles/VR.dir/geometry/Vertex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/geometry/Vertex.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/geometry/Vertex.cpp -o CMakeFiles/VR.dir/geometry/Vertex.cpp.s

CMakeFiles/VR.dir/render/BufferObject.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/BufferObject.cpp.o: ../render/BufferObject.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/VR.dir/render/BufferObject.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/BufferObject.cpp.o -c /home/lidan/Desktop/CG/CG/render/BufferObject.cpp

CMakeFiles/VR.dir/render/BufferObject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/BufferObject.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/BufferObject.cpp > CMakeFiles/VR.dir/render/BufferObject.cpp.i

CMakeFiles/VR.dir/render/BufferObject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/BufferObject.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/BufferObject.cpp -o CMakeFiles/VR.dir/render/BufferObject.cpp.s

CMakeFiles/VR.dir/render/Cam3D.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/Cam3D.cpp.o: ../render/Cam3D.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/VR.dir/render/Cam3D.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/Cam3D.cpp.o -c /home/lidan/Desktop/CG/CG/render/Cam3D.cpp

CMakeFiles/VR.dir/render/Cam3D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/Cam3D.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/Cam3D.cpp > CMakeFiles/VR.dir/render/Cam3D.cpp.i

CMakeFiles/VR.dir/render/Cam3D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/Cam3D.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/Cam3D.cpp -o CMakeFiles/VR.dir/render/Cam3D.cpp.s

CMakeFiles/VR.dir/render/EulerAngles.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/EulerAngles.cpp.o: ../render/EulerAngles.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/VR.dir/render/EulerAngles.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/EulerAngles.cpp.o -c /home/lidan/Desktop/CG/CG/render/EulerAngles.cpp

CMakeFiles/VR.dir/render/EulerAngles.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/EulerAngles.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/EulerAngles.cpp > CMakeFiles/VR.dir/render/EulerAngles.cpp.i

CMakeFiles/VR.dir/render/EulerAngles.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/EulerAngles.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/EulerAngles.cpp -o CMakeFiles/VR.dir/render/EulerAngles.cpp.s

CMakeFiles/VR.dir/render/FrameBuffer.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/FrameBuffer.cpp.o: ../render/FrameBuffer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/VR.dir/render/FrameBuffer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/FrameBuffer.cpp.o -c /home/lidan/Desktop/CG/CG/render/FrameBuffer.cpp

CMakeFiles/VR.dir/render/FrameBuffer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/FrameBuffer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/FrameBuffer.cpp > CMakeFiles/VR.dir/render/FrameBuffer.cpp.i

CMakeFiles/VR.dir/render/FrameBuffer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/FrameBuffer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/FrameBuffer.cpp -o CMakeFiles/VR.dir/render/FrameBuffer.cpp.s

CMakeFiles/VR.dir/render/Quaternion.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/Quaternion.cpp.o: ../render/Quaternion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/VR.dir/render/Quaternion.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/Quaternion.cpp.o -c /home/lidan/Desktop/CG/CG/render/Quaternion.cpp

CMakeFiles/VR.dir/render/Quaternion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/Quaternion.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/Quaternion.cpp > CMakeFiles/VR.dir/render/Quaternion.cpp.i

CMakeFiles/VR.dir/render/Quaternion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/Quaternion.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/Quaternion.cpp -o CMakeFiles/VR.dir/render/Quaternion.cpp.s

CMakeFiles/VR.dir/render/RenderWindow.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/RenderWindow.cpp.o: ../render/RenderWindow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/VR.dir/render/RenderWindow.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/RenderWindow.cpp.o -c /home/lidan/Desktop/CG/CG/render/RenderWindow.cpp

CMakeFiles/VR.dir/render/RenderWindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/RenderWindow.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/RenderWindow.cpp > CMakeFiles/VR.dir/render/RenderWindow.cpp.i

CMakeFiles/VR.dir/render/RenderWindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/RenderWindow.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/RenderWindow.cpp -o CMakeFiles/VR.dir/render/RenderWindow.cpp.s

CMakeFiles/VR.dir/render/Sample2D.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/Sample2D.cpp.o: ../render/Sample2D.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/VR.dir/render/Sample2D.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/Sample2D.cpp.o -c /home/lidan/Desktop/CG/CG/render/Sample2D.cpp

CMakeFiles/VR.dir/render/Sample2D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/Sample2D.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/Sample2D.cpp > CMakeFiles/VR.dir/render/Sample2D.cpp.i

CMakeFiles/VR.dir/render/Sample2D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/Sample2D.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/Sample2D.cpp -o CMakeFiles/VR.dir/render/Sample2D.cpp.s

CMakeFiles/VR.dir/render/Shader.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/Shader.cpp.o: ../render/Shader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/VR.dir/render/Shader.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/Shader.cpp.o -c /home/lidan/Desktop/CG/CG/render/Shader.cpp

CMakeFiles/VR.dir/render/Shader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/Shader.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/Shader.cpp > CMakeFiles/VR.dir/render/Shader.cpp.i

CMakeFiles/VR.dir/render/Shader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/Shader.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/Shader.cpp -o CMakeFiles/VR.dir/render/Shader.cpp.s

CMakeFiles/VR.dir/render/ShaderProgram.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/ShaderProgram.cpp.o: ../render/ShaderProgram.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/VR.dir/render/ShaderProgram.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/ShaderProgram.cpp.o -c /home/lidan/Desktop/CG/CG/render/ShaderProgram.cpp

CMakeFiles/VR.dir/render/ShaderProgram.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/ShaderProgram.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/ShaderProgram.cpp > CMakeFiles/VR.dir/render/ShaderProgram.cpp.i

CMakeFiles/VR.dir/render/ShaderProgram.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/ShaderProgram.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/ShaderProgram.cpp -o CMakeFiles/VR.dir/render/ShaderProgram.cpp.s

CMakeFiles/VR.dir/render/Timer.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/Timer.cpp.o: ../render/Timer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object CMakeFiles/VR.dir/render/Timer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/Timer.cpp.o -c /home/lidan/Desktop/CG/CG/render/Timer.cpp

CMakeFiles/VR.dir/render/Timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/Timer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/Timer.cpp > CMakeFiles/VR.dir/render/Timer.cpp.i

CMakeFiles/VR.dir/render/Timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/Timer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/Timer.cpp -o CMakeFiles/VR.dir/render/Timer.cpp.s

CMakeFiles/VR.dir/render/VertexArrayObject.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/VertexArrayObject.cpp.o: ../render/VertexArrayObject.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object CMakeFiles/VR.dir/render/VertexArrayObject.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/VertexArrayObject.cpp.o -c /home/lidan/Desktop/CG/CG/render/VertexArrayObject.cpp

CMakeFiles/VR.dir/render/VertexArrayObject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/VertexArrayObject.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/VertexArrayObject.cpp > CMakeFiles/VR.dir/render/VertexArrayObject.cpp.i

CMakeFiles/VR.dir/render/VertexArrayObject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/VertexArrayObject.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/VertexArrayObject.cpp -o CMakeFiles/VR.dir/render/VertexArrayObject.cpp.s

CMakeFiles/VR.dir/render/Window.cpp.o: CMakeFiles/VR.dir/flags.make
CMakeFiles/VR.dir/render/Window.cpp.o: ../render/Window.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object CMakeFiles/VR.dir/render/Window.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/VR.dir/render/Window.cpp.o -c /home/lidan/Desktop/CG/CG/render/Window.cpp

CMakeFiles/VR.dir/render/Window.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VR.dir/render/Window.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lidan/Desktop/CG/CG/render/Window.cpp > CMakeFiles/VR.dir/render/Window.cpp.i

CMakeFiles/VR.dir/render/Window.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VR.dir/render/Window.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lidan/Desktop/CG/CG/render/Window.cpp -o CMakeFiles/VR.dir/render/Window.cpp.s

# Object files for target VR
VR_OBJECTS = \
"CMakeFiles/VR.dir/main.cpp.o" \
"CMakeFiles/VR.dir/external/glad/src/glad.cpp.o" \
"CMakeFiles/VR.dir/math/MatrixCalculation.cpp.o" \
"CMakeFiles/VR.dir/geometry/Rectangle.cpp.o" \
"CMakeFiles/VR.dir/geometry/Size.cpp.o" \
"CMakeFiles/VR.dir/geometry/SpacePartition.cpp.o" \
"CMakeFiles/VR.dir/geometry/Vertex.cpp.o" \
"CMakeFiles/VR.dir/render/BufferObject.cpp.o" \
"CMakeFiles/VR.dir/render/Cam3D.cpp.o" \
"CMakeFiles/VR.dir/render/EulerAngles.cpp.o" \
"CMakeFiles/VR.dir/render/FrameBuffer.cpp.o" \
"CMakeFiles/VR.dir/render/Quaternion.cpp.o" \
"CMakeFiles/VR.dir/render/RenderWindow.cpp.o" \
"CMakeFiles/VR.dir/render/Sample2D.cpp.o" \
"CMakeFiles/VR.dir/render/Shader.cpp.o" \
"CMakeFiles/VR.dir/render/ShaderProgram.cpp.o" \
"CMakeFiles/VR.dir/render/Timer.cpp.o" \
"CMakeFiles/VR.dir/render/VertexArrayObject.cpp.o" \
"CMakeFiles/VR.dir/render/Window.cpp.o"

# External object files for target VR
VR_EXTERNAL_OBJECTS =

VR: CMakeFiles/VR.dir/main.cpp.o
VR: CMakeFiles/VR.dir/external/glad/src/glad.cpp.o
VR: CMakeFiles/VR.dir/math/MatrixCalculation.cpp.o
VR: CMakeFiles/VR.dir/geometry/Rectangle.cpp.o
VR: CMakeFiles/VR.dir/geometry/Size.cpp.o
VR: CMakeFiles/VR.dir/geometry/SpacePartition.cpp.o
VR: CMakeFiles/VR.dir/geometry/Vertex.cpp.o
VR: CMakeFiles/VR.dir/render/BufferObject.cpp.o
VR: CMakeFiles/VR.dir/render/Cam3D.cpp.o
VR: CMakeFiles/VR.dir/render/EulerAngles.cpp.o
VR: CMakeFiles/VR.dir/render/FrameBuffer.cpp.o
VR: CMakeFiles/VR.dir/render/Quaternion.cpp.o
VR: CMakeFiles/VR.dir/render/RenderWindow.cpp.o
VR: CMakeFiles/VR.dir/render/Sample2D.cpp.o
VR: CMakeFiles/VR.dir/render/Shader.cpp.o
VR: CMakeFiles/VR.dir/render/ShaderProgram.cpp.o
VR: CMakeFiles/VR.dir/render/Timer.cpp.o
VR: CMakeFiles/VR.dir/render/VertexArrayObject.cpp.o
VR: CMakeFiles/VR.dir/render/Window.cpp.o
VR: CMakeFiles/VR.dir/build.make
VR: external/imgui/libimgui.a
VR: /usr/local/lib/libspdlog.a
VR: CMakeFiles/VR.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Linking CXX executable VR"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/VR.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/VR.dir/build: VR

.PHONY : CMakeFiles/VR.dir/build

CMakeFiles/VR.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/VR.dir/cmake_clean.cmake
.PHONY : CMakeFiles/VR.dir/clean

CMakeFiles/VR.dir/depend:
	cd /home/lidan/Desktop/CG/CG/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lidan/Desktop/CG/CG /home/lidan/Desktop/CG/CG /home/lidan/Desktop/CG/CG/cmake-build-debug /home/lidan/Desktop/CG/CG/cmake-build-debug /home/lidan/Desktop/CG/CG/cmake-build-debug/CMakeFiles/VR.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/VR.dir/depend

