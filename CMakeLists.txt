cmake_minimum_required(VERSION 3.14)
project(CG)


set(CMAKE_CXX_STANDARD 14)


include(cmake/LidanBase.cmake)
include(cmake/LidanBuild.cmake)


if(NOT TARGET spdlog)
    find_package(spdlog REQUIRED)
endif()
find_package(glfw3 REQUIRED)



ADD_SUBSRC(geometrydir geometry)
ADD_SUBSRC(mathdir math)
ADD_SUBSRC(renderdir render)
ADD_SUBSRC(toolsdir tools)
ADD_SUBSRC(simulationdir simulation)

include_directories(
        math/
        geometry/
        render/
        tools/
        simulation/
        external/glad/include/
        external/SDL
        external/glm/
        external/eigen/
        external/stb/
        external/imgui/
        external/imgui/examples)

add_subdirectory(external/imgui/)
add_subdirectory(external/glm)


IF (WIN32)
    include_directories(external/SDL/lib-vc2019/include/)
    link_directories(external/SDL/lib-vc2019/lib/x64)
    message(STATUS "using windows external")
else()
    include_directories(/usr/local/Cellar/sdl2/2.0.9_1/include/)
    link_directories(/usr/local/Cellar/sdl2/2.0.9_1/lib)
    message(STATUS "using mac/linux external")
endif()




add_executable(CG
        main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/glad/src/glad.cpp
        ${mathdir}
        ${toolsdir}
        ${geometrydir}
        ${renderdir}
        ${simulationdir}
)


target_link_libraries(CG imgui SDL2 glfw spdlog::spdlog)







