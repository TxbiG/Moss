# Root
set(EXAMPLE_ROOT ${REPO_ROOT}/examples)

# List of example source files (without extension)
set(EXAMPLE_NAMES FlappyBird forest platformer spacefighter)

foreach(example_src ${EXAMPLE_NAMES})
    set(source_file ${EXAMPLE_ROOT}/${example_src}.cpp)

    if (EXISTS ${source_file})
        add_executable(${example_src} ${source_file})

        target_include_directories(${example_src} PUBLIC ${REPO_ROOT}/Moss)
        target_link_libraries(${example_src} PRIVATE Moss)

        # Enable dll import (for __declspec(dllimport)) if using shared lib
        #target_compile_definitions(${example_src} PRIVATE MOSS_SHARED_LIBRARY)

        if (MSVC)
            target_link_options(${example_src} PRIVATE "/SUBSYSTEM:CONSOLE")
        endif()

        if (EMSCRIPTEN)
            target_link_options(${example_src} PRIVATE -sSTACK_SIZE=1048576 -sINITIAL_MEMORY=134217728)
        endif()

        # Optional: Help IDEs like Visual Studio organize the source files
        source_group(TREE ${EXAMPLE_ROOT} FILES ${source_file})
    else()
        message(WARNING "Missing source file: ${source_file}. Skipping target.")
    endif()
endforeach()
