cmake_minimum_required(VERSION 3.10)
project(small_point_lio)

## use c++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# enable color output
set(CMAKE_COLOR_DIAGNOSTICS ON)

## export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# enable compile warning
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
elseif(MSVC)
    add_compile_options("/W4")
    add_compile_options("/wd4100")
endif()

# fix issues in MSVC
if(MSVC)
    add_compile_options("/utf-8")
endif()

# optimize performance
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options("-march=native")
    add_compile_options("-ffast-math")
    add_compile_options("-fno-math-errno")
endif()

##################
## 通信层开关   ##
##################
#
# 算法核心（spl_core）永远不链接 ROS。这两个开关决定把哪些通信层编进来：
#
#   -DSPL_WITH_ROS2=ON  -DSPL_WITH_STANDALONE=ON   两层都编（默认，colcon 构建）
#   -DSPL_WITH_ROS2=OFF -DSPL_WITH_STANDALONE=ON   纯 CMake 构建，产物完全不依赖 ROS，
#                                                   给没有 ROS 的机器 / 交叉编译到 ARM 用
#
# 编进来多个时，运行期由配置文件里的 `transport:` 字段决定用哪一个。
option(SPL_WITH_ROS2 "编入 ROS 2 通信层" ON)
option(SPL_WITH_STANDALONE "编入非 ROS 通信层" ON)

if(NOT SPL_WITH_ROS2 AND NOT SPL_WITH_STANDALONE)
    message(FATAL_ERROR "SPL_WITH_ROS2 和 SPL_WITH_STANDALONE 至少要开一个")
endif()

#######################
## Find dependencies ##
#######################

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# MCAP —— 录制/回放用。录出来的是标准 rosbag2，ros2 bag play / Foxglove 直接能用。
#
# ROS 2 Jazzy 的 mcap_vendor 里【只有 .hpp 没有 .inl】，所以不能定义 MCAP_IMPLEMENTATION
# 走 header-only，必须链接它编好的 libmcap.so。别改成 header-only，会报找不到 types.inl。
find_path(MCAP_INCLUDE_DIR mcap/writer.hpp
    HINTS /opt/ros/$ENV{ROS_DISTRO}/include/mcap_vendor /usr/local/include /usr/include)
find_library(MCAP_LIBRARY mcap
    HINTS /opt/ros/$ENV{ROS_DISTRO}/lib /usr/local/lib /usr/lib)
if(MCAP_INCLUDE_DIR AND MCAP_LIBRARY)
    set(SPL_MCAP_FOUND TRUE)
    message(STATUS "Found mcap: ${MCAP_LIBRARY}")
else()
    set(SPL_MCAP_FOUND FALSE)
    message(WARNING "找不到 mcap，录制与离线回放不可用")
endif()

if(SPL_WITH_STANDALONE AND NOT SPL_MCAP_FOUND)
    message(FATAL_ERROR
        "SPL_WITH_STANDALONE 需要 mcap（回放的数据来源）。\n"
        "  Ubuntu + ROS 2: sudo apt install ros-$ENV{ROS_DISTRO}-mcap-vendor\n"
        "  或者自己装 mcap 后用 -DMCAP_INCLUDE_DIR= -DMCAP_LIBRARY= 指过来")
endif()
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(SPL_WITH_ROS2)
    find_package(ament_cmake_auto REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_components REQUIRED)
    find_package(rclpy REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(livox_ros_driver2 QUIET)
    if(livox_ros_driver2_FOUND)
        message(STATUS "Found livox_ros_driver2, enabling Livox lidar support.")
        add_definitions(-DHAVE_LIVOX_DRIVER)
    else()
        message(WARNING "livox_ros_driver2 not found. Building without Livox lidar support.")
    endif()
endif()

###########
## Build ##
###########

configure_file(include/param_deliver.h.in include/param_deliver.h @ONLY)

# ---------------------------------------------------------------------------
# spl_core —— 算法核心。零 ROS 依赖，这是能搬到 32 位 ARM 板子上的那一半。
# 想验证它真的不依赖 ROS，直接 -DSPL_WITH_ROS2=OFF 用纯 CMake 构建即可。
# ---------------------------------------------------------------------------
set(SPL_CORE_SOURCES
    src/common/log.cpp
    src/io/pcd_io.cpp
    src/small_point_lio/estimator.cpp
    src/small_point_lio/parameters.cpp
    src/small_point_lio/preprocess.cpp
    src/small_point_lio/small_point_lio.cpp
    src/transport/config_loader.cpp
    src/util/pointcloud_mapping.cpp
    src/util/voxelgrid_sampling.cpp
    3rdparty/liblzf/lzf_c.c
    3rdparty/liblzf/lzf_d.c
)
add_library(spl_core STATIC ${SPL_CORE_SOURCES})
set_target_properties(spl_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(spl_core PUBLIC
    include
    3rdparty
    src
    ${CMAKE_CURRENT_BINARY_DIR}/include
)
target_link_libraries(spl_core PUBLIC Eigen3::Eigen)
target_precompile_headers(spl_core PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/include/pch.h>")
if(SPL_WITH_ROS2)
    target_compile_definitions(spl_core PUBLIC SPL_WITH_ROS2)
endif()
if(SPL_WITH_STANDALONE)
    target_compile_definitions(spl_core PUBLIC SPL_WITH_STANDALONE)
endif()

# ---------------------------------------------------------------------------
# 录制 / 回放（依赖 mcap，所以不放进 spl_core）
# ---------------------------------------------------------------------------
if(SPL_MCAP_FOUND)
    add_library(spl_recording STATIC src/transport/recording.cpp)
    set_target_properties(spl_recording PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_include_directories(spl_recording PUBLIC ${MCAP_INCLUDE_DIR})
    target_link_libraries(spl_recording PUBLIC spl_core ${MCAP_LIBRARY})
    target_compile_definitions(spl_recording PUBLIC SPL_WITH_MCAP)
endif()

# ---------------------------------------------------------------------------
# 非 ROS 通信层
# ---------------------------------------------------------------------------
if(SPL_WITH_STANDALONE)
    add_library(spl_transport_standalone STATIC
        src/transport/standalone/standalone_transport.cpp
    )
    set_target_properties(spl_transport_standalone PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_link_libraries(spl_transport_standalone PUBLIC spl_core spl_recording)
endif()

# ---------------------------------------------------------------------------
# ROS 2 通信层
# ---------------------------------------------------------------------------
if(SPL_WITH_ROS2)
    ament_auto_add_library(${PROJECT_NAME} SHARED
        src/transport/ros2/ros_config.cpp
        src/transport/ros2/ros_transport.cpp
    )
    target_link_libraries(${PROJECT_NAME} spl_core)
    if(SPL_MCAP_FOUND)
        target_link_libraries(${PROJECT_NAME} spl_recording)
    endif()
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rclcpp_components
        rclpy
        sensor_msgs
        geometry_msgs
        nav_msgs
        tf2_ros
        tf2_geometry_msgs
        std_srvs
    )
    if(livox_ros_driver2_FOUND)
        ament_target_dependencies(${PROJECT_NAME}
            livox_ros_driver2
        )
    endif()

    rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN small_point_lio::SmallPointLioNode
        EXECUTABLE ${PROJECT_NAME}_node
    )
endif()

# ---------------------------------------------------------------------------
# 统一入口可执行文件：读配置 -> 按 transport 字段选通信层 -> 跑
# transport.cpp（工厂）编在这里，这样它能同时看见编进来的各个通信层。
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_standalone
    src/app/standalone_main.cpp
    src/transport/transport.cpp
)
target_link_libraries(${PROJECT_NAME}_standalone spl_core)
if(SPL_WITH_STANDALONE)
    target_link_libraries(${PROJECT_NAME}_standalone spl_transport_standalone)
endif()
if(SPL_WITH_ROS2)
    target_link_libraries(${PROJECT_NAME}_standalone ${PROJECT_NAME})
    ament_target_dependencies(${PROJECT_NAME}_standalone rclcpp)
endif()

#############
## Testing ##
#############

if(SPL_WITH_ROS2 AND BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
        ament_cmake_copyright
        ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

if(SPL_WITH_ROS2)
    install(TARGETS ${PROJECT_NAME}_standalone
        DESTINATION lib/${PROJECT_NAME}
    )
    ament_auto_package(
        USE_SCOPED_HEADER_INSTALL_DIR
        INSTALL_TO_SHARE
        config
        launch
    )
else()
    install(TARGETS ${PROJECT_NAME}_standalone
        RUNTIME DESTINATION bin
    )
endif()
