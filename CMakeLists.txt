cmake_minimum_required(VERSION 2.6)
project(clangtest)
set (CMAKE_CXX_STANDARD 11)
include_directories(/usr/lib/llvm-4.0/include/)
link_directories(/usr/lib/llvm-4.0/lib/)

add_executable(clangtest main.cpp)

target_link_libraries(clangtest clang-4.0)

install(TARGETS clangtest RUNTIME DESTINATION bin)

add_custom_target(test ./clangtest ${CMAKE_SOURCE_DIR}/test/tdot_asguard.cpp -DBASE_LOG_NAMESPACE=taskmanagement -DOROCOS_TARGET=gnulinux -DQT_CORE_LIB -DQT_GUI_LIB -DQT_NO_DEBUG -DQT_SHARED -D_INFO 
    -D__OSVERSION__=2 -D__linux__ -D__x86_64__ 
    -I${CMAKE_SOURCE_DIR}/test/include 
    -I${CMAKE_SOURCE_DIR}/test/include/eigen3 
    -I${CMAKE_SOURCE_DIR}/test/include/orocos 
    -I${CMAKE_SOURCE_DIR}/test/include/pcl-1.8
    -fuse-ld=gold -std=c++11 
    -DOROCOS_TARGET=gnulinux -std=c++11 -lpthread -O2 -g -DNDEBUG   -Wall -Wno-unused-local-typedefs
    -Wnon-virtual-dtor
    DEPENDS 
        clangtest)

add_custom_target(test_pre ./clangtest ${CMAKE_SOURCE_DIR}/test/preprocessed.cpp -DBASE_LOG_NAMESPACE=taskmanagement -DOROCOS_TARGET=gnulinux -DQT_CORE_LIB -DQT_GUI_LIB -DQT_NO_DEBUG -DQT_SHARED -D_INFO 
    -D__OSVERSION__=2 -D__linux__ -D__x86_64__ 
    -I${CMAKE_SOURCE_DIR}/test/include 
    -I${CMAKE_SOURCE_DIR}/test/include/eigen3 
    -I${CMAKE_SOURCE_DIR}/test/include/orocos 
    -I${CMAKE_SOURCE_DIR}/test/include/pcl-1.8
    -fuse-ld=gold -std=c++11 
    -DOROCOS_TARGET=gnulinux -std=c++11 -lpthread -O2 -g -DNDEBUG   -Wall -Wno-unused-local-typedefs
    -Wnon-virtual-dtor
    DEPENDS 
        clangtest)
        
add_custom_target(gen_preprocess clang -E ${CMAKE_SOURCE_DIR}/test/tdot_asguard.cpp -DBASE_LOG_NAMESPACE=taskmanagement -DOROCOS_TARGET=gnulinux -DQT_CORE_LIB -DQT_GUI_LIB -DQT_NO_DEBUG -DQT_SHARED -D_INFO 
    -D__OSVERSION__=2 -D__linux__ -D__x86_64__ 
    -I${CMAKE_SOURCE_DIR}/test/include 
    -I${CMAKE_SOURCE_DIR}/test/include/eigen3 
    -I${CMAKE_SOURCE_DIR}/test/include/orocos 
    -I${CMAKE_SOURCE_DIR}/test/include/pcl-1.8
    -fuse-ld=gold -std=c++11 
    -DOROCOS_TARGET=gnulinux -std=c++11 -lpthread -O2 -g -DNDEBUG   -Wall -Wno-unused-local-typedefs
    -Wnon-virtual-dtor -o ${CMAKE_SOURCE_DIR}/test/preprocessed.cpp)

