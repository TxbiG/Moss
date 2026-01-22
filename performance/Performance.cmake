# Root
set(PERFORMANCE_TEST_ROOT ${REPO_ROOT}/performance)

# Source files
set(PERFORMANCE_SRC_FILES
    ${PERFORMANCE_TEST_ROOT}/main.cpp
)

# Group source files (for IDEs)
source_group(TREE ${PERFORMANCE_TEST_ROOT} FILES ${PERFORMANCE_SRC_FILES})

add_executable(performance_test WIN32 ${PERFORMANCE_SRC_FILES})

# Compile definitions
#target_compile_definitions(performance_test PRIVATE MOSS_STATIC_LIBRARY)

# Include directories
target_include_directories(performance_test PUBLIC 
    ${PERFORMANCE_TEST_ROOT}       # glad.h lives here
    ${REPO_ROOT}/Moss
)

# Link libraries
#if(USE_OPENGL)
target_link_libraries(performance_test PRIVATE Moss 
    opengl32
    user32
    gdi32)
#endif()