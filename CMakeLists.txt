cmake_minimum_required(VERSION 3.14)
project(carm_tool LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # 设置为当前构建目录下的 install 文件夹
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/../install" CACHE PATH "Installation directory" FORCE)
endif()

find_package(Threads REQUIRED)
find_package(jsoncpp REQUIRED)

# 库
add_library(${PROJECT_NAME} SHARED 
    src/carm_tool.cpp
)
target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        jsoncpp
    PRIVATE
        Threads::Threads
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 公开自己的头文件
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    PRIVATE
        ${jsoncpp_INCLUDE_DIRS}
)

# 运行文件
add_executable(gripper_demo_node src/gripper_demo.cpp)
target_link_libraries(gripper_demo_node 
    PRIVATE 
        ${PROJECT_NAME}
        Threads::Threads
)
target_include_directories(gripper_demo_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(handle_demo_node src/handle_demo.cpp)
target_link_libraries(handle_demo_node
    PRIVATE
        ${PROJECT_NAME}
        Threads::Threads
)
target_include_directories(handle_demo_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(screwdriver_demo_node src/screwdriver_demo.cpp)
target_link_libraries(screwdriver_demo_node 
    PRIVATE 
        ${PROJECT_NAME}
        Threads::Threads
)
target_include_directories(screwdriver_demo_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)


install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib          # .so 文件
    ARCHIVE DESTINATION lib          # .a 文件
    RUNTIME DESTINATION bin          # Windows .dll 文件
    INCLUDES DESTINATION include     # 公共头文件
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/. DESTINATION include
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN "*.hxx"
)

install(TARGETS 
    gripper_demo_node 
    handle_demo_node
    screwdriver_demo_node
    RUNTIME DESTINATION bin
)
