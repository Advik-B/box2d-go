# CMake file to build only the Box2D static library for CGO
cmake_minimum_required(VERSION 3.22)

project(box2d_static LANGUAGES C)

# Deterministic math
if (MINGW OR APPLE OR UNIX)
	add_compile_options(-ffp-contract=off)
endif()

# Box2D source files from submodule
file(GLOB BOX2D_SOURCE_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/../../box2d/src/*.c
)

# Create static library
add_library(box2d STATIC ${BOX2D_SOURCE_FILES})

# Include directories from submodule
target_include_directories(box2d PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/../../box2d/include
	${CMAKE_CURRENT_SOURCE_DIR}/../../box2d/src
)

# Set C standard
set_target_properties(box2d PROPERTIES
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_EXTENSIONS NO
	POSITION_INDEPENDENT_CODE ON
)

# Add definitions for time support
target_compile_definitions(box2d PRIVATE _POSIX_C_SOURCE=199309L)

# Link math library on Unix
if(UNIX)
	target_link_libraries(box2d PUBLIC m)
endif()
