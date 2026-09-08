cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME moteus_test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# target_sources(${CMAKE_PROJECT_NAME} PRIVATE
#     # Add user sources here
# )
target_sources(moteus_test PRIVATE
    # ─────────────────────────────────────────────────────────────────────
    # NORMAL OPERATION:  use main.c (FOC system)
    # CALIBRATION MODE:  comment out main.c above and uncomment
    #                    encoder_calibration.c below.
    #                    Only ONE of the two may be active at a time —
    #                    both files define their own int main(void).
    # See README.md  > "Encoder calibration" for the full procedure.
    # ─────────────────────────────────────────────────────────────────────
    # Core/Src/main.c
    Core/Src/encoder_calibration.c

    # CubeMX-generated (do not modify)
    Core/Src/stm32g4xx_it.c
    Core/Src/stm32g4xx_hal_msp.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c

    # Student-implemented modules
    Core/Src/controller.c
    Core/Src/svpwm.c
    Core/Src/encoder.c
    Core/Src/drv8323.c
    Core/Src/cascade_foc.c
    Core/Src/current_adc.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
