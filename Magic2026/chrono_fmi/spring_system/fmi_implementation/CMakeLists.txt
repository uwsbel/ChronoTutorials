cmake_minimum_required(VERSION 3.18)
project(two_mass_spring_fmu)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# --- Find the STATIC Chrono build (configure with Chrono_DIR pointing at chrono_build_static/cmake) ---
find_package(Chrono COMPONENTS FMI REQUIRED CONFIG)

# fmu-forge path comes from the Chrono config
if(NOT DEFINED CHRONO_FMU_FORGE_DIR)
  message(FATAL_ERROR "CHRONO_FMU_FORGE_DIR not defined. Is this the static Chrono build with FMU export support?")
endif()
set(FMU_FORGE_DIR ${CHRONO_FMU_FORGE_DIR})

# --- FMU configuration (the fmu-forge contract) ---
set(FMU_MODEL_IDENTIFIER "FMU3cs_two_mass_spring")
set(FMU_CS TRUE)
set(USE_CUSTOM_TYPESPLATFORM OFF)
set(FMU_MAIN_DIRECTORY "${CMAKE_BINARY_DIR}")   # .fmu will go under here
set(FMU_MSG_PREFIX "   - ")

include_directories(${FMU_FORGE_DIR})

# --- Bring in the fmu-forge fmi3 machinery (creates the FMU target + packaging) ---
FetchContent_Declare(
    ${FMU_MODEL_IDENTIFIER}
    SOURCE_DIR ${FMU_FORGE_DIR}/fmi3
)
FetchContent_MakeAvailable(${FMU_MODEL_IDENTIFIER})

# --- Attach YOUR sources to the FMU target fmu-forge created ---
add_dependencies(${FMU_MODEL_IDENTIFIER} Chrono::Chrono_core)
target_sources(${FMU_MODEL_IDENTIFIER} PRIVATE
    src/two_mass_spring_fmu.cpp
    src/two_mass_spring_fmu.h)
target_link_libraries(${FMU_MODEL_IDENTIFIER} Chrono::Chrono_core)

# --- CRITICAL: static MSVC runtime (/MT), must match Chrono static build ---
if(MSVC)
  set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES
      MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

message(STATUS "FMU will be created at: ${FMU_FILENAME}")