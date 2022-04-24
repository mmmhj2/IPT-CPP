cmake_minimum_required(VERSION 3.15)  # CMake version check
project(IPT_cpp)  # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)  # Enable c++14 standard

find_package(OpenCV REQUIRED)
find_package(Threads)

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
#find_package(apriltag REQUIRED)
#find_package(nlohmann_json REQUIRED)

# 查找头文件。预编译：将include文件对应的内容替换#include
include_directories(${OpenCV_INCLUDE_DIRS}, include)

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_ROOT)
aux_source_directory(./src DIR_SRC)
aux_source_directory(./include DIR_INCLUDE)


# 指定生成目标，可以指定多个. 有executable和library等等。
#link_libraries(CNPY)  # 静态库在add_executable之前导入，导入库在add_executable之后导入。
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")
link_directories(${apriltag_LIBDIR})
include_directories(include ${apriltag_INCLUDE_DIRS})

add_executable(IPT_cpp main.cpp ${DIR_SRC} ${DIR_INCLUDE})

# make sure to place target_link_libraries after the add_executable command,
# so that CMake actually builds the target before linking the library.
target_link_libraries(IPT_cpp ${apriltag_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT}) # link OpenCV libraries
#MESSAGE("OpenCV_LIBS: " ${OpenCV_LIBS})  #display opencv libs found

add_executable(record_video record_video.c src/avilib.c src/v4l2uvc.c include/avilib.h include/uvcvideo.h include/v4l2uvc.h)