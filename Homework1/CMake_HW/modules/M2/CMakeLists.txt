set(M2_INCLUDE_DIR)
list(APPEND M2_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")

set(M2_INCLUDE_DIRS ${M2_INCLUDE_DIR}
                    CACHE PATH "M2 module include directories")

add_library(M2 STATIC
    src/M2.cpp
)

target_include_directories(M2
    PUBLIC
        ${M2_INCLUDE_DIRS}
)

target_link_libraries(M2 PUBLIC kalman A1 A2)