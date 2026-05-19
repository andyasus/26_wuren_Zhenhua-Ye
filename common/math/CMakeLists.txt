# 新增：找到OpenCV库
find_package(OpenCV REQUIRED)

set(math_INCLUDE_DIR)
list(APPEND math_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")
# 新增：把OpenCV的头文件目录加进来
list(APPEND math_INCLUDE_DIR "${OpenCV_INCLUDE_DIRS}")

set(math_INCLUDE_DIRS ${math_INCLUDE_DIR}
                      CACHE PATH "math utility include directories")

add_library(math STATIC
    src/Math.cpp
)

target_include_directories(math
    PUBLIC
        ${math_INCLUDE_DIRS}
)

target_link_libraries(math PUBLIC ${OpenCV_LIBS})