include(FetchContent)

function(FetchContent_Declare_Fallback)
    set(FETCH_CONTENT_ARGS ${ARGN})

    if(CMAKE_VERSION VERSION_LESS "3.24.0")
        list(FIND FETCH_CONTENT_ARGS "FIND_PACKAGE_ARGS" FIND_PACKAGE_ARGS_INDEX)

        if(NOT (FIND_PACKAGE_ARGS_INDEX STREQUAL "-1"))
            set(LAST_ELEMENT "")
            while(NOT (LAST_ELEMENT STREQUAL "FIND_PACKAGE_ARGS"))
                list(POP_BACK FETCH_CONTENT_ARGS LAST_ELEMENT)
            endwhile()
        endif()
    endif()

    if(CMAKE_VERSION VERSION_LESS "3.25.0")
        list(REMOVE_ITEM FETCH_CONTENT_ARGS SYSTEM)
    endif()

    FetchContent_Declare(${FETCH_CONTENT_ARGS})
endfunction()