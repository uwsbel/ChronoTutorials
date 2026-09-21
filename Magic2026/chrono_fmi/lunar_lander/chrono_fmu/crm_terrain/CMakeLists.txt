# external_solver/CMakeLists.txt

cmake_minimum_required(VERSION 3.18)
project(crm_terrain)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

set(MY_FILES src/crm_terrain.cpp src/crm_terrain.h)
set(EXEC_NAME crm_terrain)
add_executable(${EXEC_NAME} ${MY_FILES})

find_package(cppzmq CONFIG REQUIRED)
find_package(ZeroMQ CONFIG REQUIRED)

find_package(Chrono COMPONENTS VSG FSI VEHICLE REQUIRED CONFIG)

target_include_directories(${EXEC_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../common)
target_link_libraries(${EXEC_NAME} PRIVATE cppzmq libzmq)
target_link_libraries(${EXEC_NAME} PRIVATE Chrono::Chrono_core Chrono::Chrono_vsg Chrono::Chrono_vehicle Chrono::Chrono_fsi Chrono::Chrono_fsisph Chrono::Chrono_fsisph_vsg)

if(MSVC)
  set_target_properties(${EXEC_NAME} PROPERTIES
      MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

target_compile_definitions(${EXEC_NAME}  PRIVATE
    PROJECT_RESOURCES_DIR="${CMAKE_SOURCE_DIR}/../resources"
    PROJECT_RESULTS_DIR="${CMAKE_SOURCE_DIR}/../results")

target_compile_definitions(${EXEC_NAME} PRIVATE CHRONO_DATA_DIR="${CHRONO_DATA_DIR}")

# --- Define the shared resources dir (used by the FMU packaging too) ---
# Put it somewhere predictable; here: <build>/fmu_resources
set(FMU_RESOURCES_DIR "${CMAKE_SOURCE_DIR}/../resources" CACHE INTERNAL "FMU resources dir")

# --- Copy external_solver.exe into resources after it builds ---
add_custom_command(TARGET ${EXEC_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${FMU_RESOURCES_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:${EXEC_NAME}>"          # the .exe
        "${FMU_RESOURCES_DIR}/"
    COMMENT "Copying ${EXEC_NAME}.exe into FMU resources")


add_CHRONO_DLLS_copy_command() # to copy Chrono DLLs into project build

# --- Package: copy exe + ALL DLLs (whatever landed next to it) into resources ---
add_custom_command(TARGET ${EXEC_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${FMU_RESOURCES_DIR}"
    # copy the entire output directory (exe + all DLLs Chrono staged) into resources
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "$<TARGET_FILE_DIR:${EXEC_NAME}>"       # the exe's output dir (exe + DLLs)
        "${FMU_RESOURCES_DIR}"
    COMMENT "Copying ${EXEC_NAME}.exe and all DLLs into FMU resources")
