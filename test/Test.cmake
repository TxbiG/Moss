# Root
set(UNIT_TESTS_ROOT ${PHYSICS_REPO_ROOT}/performance)

# Source files
set(HELLO_WORLD_SRC_FILES
	${HELLO_WORLD_ROOT}/main.cpp
	${HELLO_WORLD_ROOT}/Performance.cmake
)

# Group source files
source_group(TREE ${HELLO_WORLD_ROOT} FILES ${HELLO_WORLD_SRC_FILES})