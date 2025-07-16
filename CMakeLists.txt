cmake_minimum_required(VERSION 3.10)
project(hello_world)
set(CMAKE_CXX_STANDARD 11)

# 指定Eigen3安装路径
set(CMAKE_PREFIX_PATH "D:/vscode/eigen/eigen")

# 查找Eigen3包
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# 手动配置KDL库路径
set(KDL_ROOT "D:/vscode/KDL/orocos_cmake")
set(KDL_INCLUDE_DIRS "${KDL_ROOT}/include")
set(KDL_LIBRARY_DIRS "${KDL_ROOT}/lib")

# 查找KDL库文件
find_library(KDL_LIBRARIES 
    NAMES orocos-kdl kdl
    PATHS ${KDL_LIBRARY_DIRS}
    NO_DEFAULT_PATH
)

# 检查是否找到KDL库
if(NOT KDL_LIBRARIES)
    message(FATAL_ERROR "Could not find KDL library in ${KDL_LIBRARY_DIRS}")
endif()

# 包含头文件目录
include_directories(${KDL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)

add_executable(hello_world ${SRC_LIST})

# 链接Eigen3库
target_link_libraries(hello_world Eigen3::Eigen)
# 链接KDL库
target_link_libraries(hello_world ${KDL_LIBRARIES})