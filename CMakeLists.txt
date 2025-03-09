cmake_minimum_required(VERSION 3.5)
project(slam_toolbox)

set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
cmake_policy(SET CMP0077 NEW)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake/")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/karto_sdk/cmake)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)

#karto_sdk lib
set(BUILD_SHARED_LIBS ON)
add_subdirectory(lib/karto_sdk)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(dependencies
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_sensor_msgs
  std_msgs
  std_srvs
  builtin_interfaces
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  interactive_markers
  Qt5
  rclcpp_components
  OpenCV
  PCL
  cv_bridge
  rmw
  rmw_implementation
)

set(libraries
    toolbox_common
    SlamToolboxPlugin
    ceres_solver_plugin
    async_slam_toolbox
    sync_slam_toolbox
    localization_slam_toolbox
    lifelong_slam_toolbox
    map_and_localization_slam_toolbox
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CHOLMOD REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Boost REQUIRED system serialization filesystem thread)

include_directories(include lib/karto_sdk/include
                            ${EIGEN3_INCLUDE_DIRS}
                            ${CHOLMOD_INCLUDE_DIR}
                            ${Boost_INCLUDE_DIRS}
                            ${TBB_INCLUDE_DIRS}
                            ${CERES_INCLUDES}
                            ${OpenCV_INCLUDE_DIRS}
                            ${PCL_INCLUDE_DIRS}
                            ${rclcpp_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})
add_definitions(${OpenCV_DEFINITIONS})

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/Pause.srv
  srv/ClearQueue.srv
  srv/ToggleInteractive.srv
  srv/Clear.srv
  srv/SaveMap.srv
  srv/LoopClosure.srv
  srv/MergeMaps.srv
  srv/AddSubmap.srv
  srv/DeserializePoseGraph.srv
  srv/SerializePoseGraph.srv
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs visualization_msgs
)

#### rviz Plugin
qt5_wrap_cpp(MOC_FILES rviz_plugin/slam_toolbox_rviz_plugin.hpp)
add_library(SlamToolboxPlugin SHARED
  rviz_plugin/slam_toolbox_rviz_plugin.cpp
  ${MOC_FILES})
ament_target_dependencies(SlamToolboxPlugin
 ${dependencies}
)
target_include_directories(SlamToolboxPlugin PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(SlamToolboxPlugin ${QT_LIBRARIES} rviz_common::rviz_common)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(SlamToolboxPlugin "${cpp_typesupport_target}")
target_compile_definitions(SlamToolboxPlugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(SlamToolboxPlugin PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml) 

#### Ceres Plugin
add_library(ceres_solver_plugin solvers/ceres_solver.cpp)
ament_target_dependencies(ceres_solver_plugin ${dependencies})
target_link_libraries(ceres_solver_plugin ${CERES_LIBRARIES}
                                          ${Boost_LIBRARIES}
                                          ${TBB_LIBRARIES}
                                          kartoSlamToolbox )

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ceres_solver_plugin "${cpp_typesupport_target}")
pluginlib_export_plugin_description_file(slam_toolbox solver_plugins.xml)

#### Tool lib for mapping
add_library(toolbox_common src/slam_toolbox_common.cpp src/map_saver.cpp src/loop_closure_assistant.cpp src/laser_utils.cpp src/slam_mapper.cpp src/ORBextractor.cpp src/camera_loop_closure_assistant.cpp src/camera_utils.cpp src/camera_feature_extraction_node.cpp)
ament_target_dependencies(toolbox_common rmw rclcpp ${dependencies})
target_link_libraries(toolbox_common kartoSlamToolbox camera_utils ORBextractor ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(toolbox_common "${cpp_typesupport_target}")

#### Mapping executibles
add_library(async_slam_toolbox src/slam_toolbox_async.cpp)
target_link_libraries(async_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(async_slam_toolbox_node src/slam_toolbox_async_node.cpp)
target_link_libraries(async_slam_toolbox_node async_slam_toolbox)

add_library(sync_slam_toolbox src/slam_toolbox_sync.cpp)
target_link_libraries(sync_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(sync_slam_toolbox_node src/slam_toolbox_sync_node.cpp)
target_link_libraries(sync_slam_toolbox_node sync_slam_toolbox)

add_library(localization_slam_toolbox src/slam_toolbox_localization.cpp)
target_link_libraries(localization_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(localization_slam_toolbox_node src/slam_toolbox_localization_node.cpp)
target_link_libraries(localization_slam_toolbox_node localization_slam_toolbox)

add_library(lifelong_slam_toolbox src/experimental/slam_toolbox_lifelong.cpp)
target_link_libraries(lifelong_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(lifelong_slam_toolbox_node src/experimental/slam_toolbox_lifelong_node.cpp)
target_link_libraries(lifelong_slam_toolbox_node lifelong_slam_toolbox)

add_library(map_and_localization_slam_toolbox src/experimental/slam_toolbox_map_and_localization.cpp)
target_link_libraries(map_and_localization_slam_toolbox localization_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(map_and_localization_slam_toolbox_node src/experimental/slam_toolbox_map_and_localization_node.cpp)
target_link_libraries(map_and_localization_slam_toolbox_node toolbox_common map_and_localization_slam_toolbox)

#### Merging maps tool
add_executable(merge_maps_kinematic src/merge_maps_kinematic.cpp)
target_link_libraries(merge_maps_kinematic kartoSlamToolbox toolbox_common)

#### New Camera Feature Libraries
add_library(camera_utils src/camera_utils.cpp)
ament_target_dependencies(camera_utils rclcpp rmw ${dependencies})
target_link_libraries(camera_utils kartoSlamToolbox ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${cpp_typesupport_target})

add_library(ORBextractor src/ORBextractor.cpp)
ament_target_dependencies(ORBextractor rclcpp ${dependencies})
target_link_libraries(ORBextractor ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_library(camera_loop_closure_assistant src/camera_loop_closure_assistant.cpp)
ament_target_dependencies(camera_loop_closure_assistant rclcpp rmw ${dependencies})
target_link_libraries(camera_loop_closure_assistant camera_utils ORBextractor kartoSlamToolbox ${OpenCV_LIBRARIES} ${cpp_typesupport_target})

add_executable(camera_feature_extraction_node src/camera_feature_extraction_node.cpp)
ament_target_dependencies(camera_feature_extraction_node rclcpp ${dependencies})
target_link_libraries(camera_feature_extraction_node camera_utils ORBextractor kartoSlamToolbox ${OpenCV_LIBRARIES})

#### testing
#if(BUILD_TESTING)
#  find_package(ament_cmake_gtest REQUIRED)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#  include_directories(test)
#  ament_add_gtest(lifelong_metrics_test test/lifelong_metrics_test.cpp)
#  target_link_libraries(lifelong_metrics_test lifelong_slam_toolbox)
#endif()

rclcpp_components_register_nodes(async_slam_toolbox "slam_toolbox::AsynchronousSlamToolbox")
rclcpp_components_register_nodes(sync_slam_toolbox "slam_toolbox::SynchronousSlamToolbox")
rclcpp_components_register_nodes(localization_slam_toolbox "slam_toolbox::LocalizationSlamToolbox")
rclcpp_components_register_nodes(lifelong_slam_toolbox "slam_toolbox::LifelongSlamToolbox")
rclcpp_components_register_nodes(map_and_localization_slam_toolbox "slam_toolbox::MapAndLocalizationSlamToolbox")

#### Install
install(TARGETS async_slam_toolbox_node
                sync_slam_toolbox_node
                localization_slam_toolbox_node
                camera_feature_extraction_node
                map_and_localization_slam_toolbox_node
                merge_maps_kinematic
                ${libraries}
                kartoSlamToolbox
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS SlamToolboxPlugin
  EXPORT SlamToolboxPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "lifelong_launch.py" EXCLUDE
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES solver_plugins.xml rviz_plugins.xml
  DESTINATION share
)

install(
  PROGRAMS
    src/tf_broadcaster_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install ORBextractor library
install(TARGETS ORBextractor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS camera_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS camera_loop_closure_assistant
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Make sure the interfaces are generated before they're used
if(TARGET ${PROJECT_NAME}_generate_messages)
  add_dependencies(async_slam_toolbox ${PROJECT_NAME}_generate_messages)
  add_dependencies(sync_slam_toolbox ${PROJECT_NAME}_generate_messages)
  add_dependencies(localization_slam_toolbox ${PROJECT_NAME}_generate_messages)
  add_dependencies(lifelong_slam_toolbox ${PROJECT_NAME}_generate_messages)
  add_dependencies(map_and_localization_slam_toolbox ${PROJECT_NAME}_generate_messages) 
  add_dependencies(camera_loop_closure_assistant ${PROJECT_NAME})
endif()


ament_export_include_directories(include)
ament_export_libraries(${libraries} kartoSlamToolbox)
ament_export_dependencies(${dependencies})
ament_export_targets(SlamToolboxPlugin HAS_LIBRARY_TARGET)
ament_package()

add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)
add_definitions(-DOPENCV_DISABLE_EIGEN_TENSOR_SUPPORT)  # If you're using Eigen

# Add these to your target properties
set_target_properties(camera_utils PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    COMPILE_FLAGS "-Wno-deprecated-declarations"
)

