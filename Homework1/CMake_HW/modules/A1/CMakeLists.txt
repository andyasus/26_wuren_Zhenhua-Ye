set(A1_INCLUDE_DIR)
list(APPEND A1_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")

set(A1_INCLUDE_DIRS ${A1_INCLUDE_DIR}
                    CACHE PATH "A1 module include directories")

add_library(A1 STATIC
    src/A11.cpp
    src/A12.cpp
    src/A13.cpp
)

target_include_directories(A1
    PUBLIC
        ${A1_INCLUDE_DIRS}
)