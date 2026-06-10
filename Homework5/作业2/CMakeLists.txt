cmake_minimum_required(VERSION 3.10)
project(RobotDogOptimization LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

find_package(osqp QUIET)

if(NOT osqp_FOUND)
    set(OSQP_INCLUDE_DIR "/tmp/osqp/include")
    set(OSQP_LIBRARY "/tmp/osqp/build/out/libosqp.so")

    if(EXISTS ${OSQP_LIBRARY} AND EXISTS ${OSQP_INCLUDE_DIR})
        add_library(osqp_from_local STATIC IMPORTED)
        set_target_properties(osqp_from_local PROPERTIES
            IMPORTED_LOCATION ${OSQP_LIBRARY}
        )
        set(osqp_FOUND TRUE)
        set(osqp_TARGET osqp_from_local)
        set(osqp_INCLUDE_DIR ${OSQP_INCLUDE_DIR})
    else()
        include(FetchContent)
        FetchContent_Declare(osqp
            GIT_REPOSITORY https://github.com/osqp/osqp.git
            GIT_TAG v0.6.3)
        FetchContent_MakeAvailable(osqp)
    endif()
endif()

if(NOT DEFINED osqp_TARGET)
    if(TARGET osqp::osqp)
        set(osqp_TARGET osqp::osqp)
    elseif(TARGET osqp)
        set(osqp_TARGET osqp)
    else()
        message(FATAL_ERROR "Cannot determine OSQP target name")
    endif()
endif()

add_executable(q1_gradient_descent q1_gradient_descent.cpp)
target_link_libraries(q1_gradient_descent Eigen3::Eigen)

add_executable(q3_osqp_solve q3_osqp_solve.cpp)
if(DEFINED osqp_INCLUDE_DIR)
    target_include_directories(q3_osqp_solve PRIVATE ${osqp_INCLUDE_DIR})
endif()
target_link_libraries(q3_osqp_solve PRIVATE ${osqp_TARGET} Eigen3::Eigen)
