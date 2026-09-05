cmake_minimum_required(VERSION 4.3)

project(GameEngine)

set(CMAKE_CXX_STANDARD 23)

include(FetchContent)

FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.5
)

FetchContent_MakeAvailable(raylib)

add_executable(GameEngine main.cpp)

target_include_directories(GameEngine PRIVATE
        externalstuff/raygui
)

target_link_libraries(GameEngine PRIVATE raylib)