set(A2_INCLUDE_DIR)
list(APPEND A2_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")

set(A2_INCLUDE_DIRS ${A2_INCLUDE_DIR}
                    CACHE PATH "A2 module include directories")

add_library(A2 STATIC
    src/A2.cpp
)

target_include_directories(A2
    PUBLIC
        ${A2_INCLUDE_DIRS}
)

target_compile_options(A2 PRIVATE -include cstddef)