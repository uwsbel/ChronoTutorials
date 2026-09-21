cmake_minimum_required(VERSION 3.18)

project(chrono_matlab_fmi_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Chrono REQUIRED CONFIG)

set(MY_FILES src/two_mass_spring.cpp)
add_executable(two_mass_spring ${MY_FILES})

if(MSVC)
  set_target_properties(two_mass_spring PROPERTIES
      MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

target_link_libraries(two_mass_spring Chrono::Chrono_core)

target_compile_definitions(two_mass_spring PRIVATE
    PROJECT_RESULTS_DIR="${CMAKE_SOURCE_DIR}/results")

add_CHRONO_DLLS_copy_command() # to copy Chrono DLLs into project build