cmake_minimum_required(VERSION 3.14)
project(CG)

set(CMAKE_CXX_STANDARD 14)

include(cmake/LidanBase.cmake)
include(cmake/LidanBuild.cmake)



ADD_SUBSRC(geometrydir geometry)
ADD_SUBSRC(mathdir math)


add_subdirectory(external/glfw)
add_subdirectory(external/imgui/)

Lidan_ADD_SRC(PATH "${CMAKE_CURRENT_SOURCE_DIR}/external/glad/" SOURCEDIR gladsrc)


include_directories(
        math/
        geometry/
        external/glfw/include
        external/glad
        external/SDL
        external/glm/
        external/eigen/
        external/imgui/
        external/imgui/examples)


IF (WIN32)
    include_directories(external/SDL/lib-vc2019/include/)
    link_directories(external/SDL/lib-vc2019/lib/x64)
    message(STATUS "using windows external")
else()
    include_directories(/usr/local/Cellar/sdl2/2.0.9_1/include/)
    link_directories(/usr/local/Cellar/sdl2/2.0.9_1/lib)
    message(STATUS "using mac/linux external")
endif()


add_executable(VR
        main.cpp
        ${mathdir}
        ${geometrydir}
        )



target_link_libraries(VR imgui glfw3 SDL2)





