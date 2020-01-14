cmake_minimum_required(VERSION 3.5)

project(ros_sensors_qt512_ndk19 LANGUAGES CXX)

#set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC TRUE)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_SHARED_LIBS OFF)
find_package(Qt5 COMPONENTS Widgets Gui Core Quick Positioning Sensors REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fastrtps_cmake_module REQUIRED)
find_package(fastrtps REQUIRED CONFIG)
find_package(tf2 REQUIRED)

include_directories(
    include/
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}

)

if(ANDROID)
    add_library(ros_sensors_qt512_ndk19 SHARED
        src/main.cpp
        include/gps_spitter.h
        src/gps_spitter.cpp
        qml.qrc)
else()
    add_executable(ros_sensors_qt512_ndk19
        src/main.cpp
        src/gps_spitter.cpp
        qml.qrc)
endif()

target_compile_definitions(ros_sensors_qt512_ndk19
  PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)

target_link_libraries(ros_sensors_qt512_ndk19
    Qt5::Core
    Qt5::Quick
    Qt5::Positioning
    Qt5::Sensors
    ${std_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${tf2_LIBRARIES}
    fastrtps
    )

ament_export_include_directories(include)
ament_export_dependencies(rclcpp builtin_interfaces std_msgs)

ament_export_libraries(
  ${PROJECT_NAME})

ament_package()
file(GLOB_RECURSE
    MY_ANDROID_LIBS_LIST
    /home/johnconn/external_storage/ros2_core_android_ws/install/device/*.so)
message("glob recurse setting to: ")

foreach(OPT ${MY_ANDROID_LIBS_LIST})
   set(MY_ANDROID_STRING "${MY_ANDROID_STRING} ${OPT}")
endforeach()
message("${MY_ANDROID_STRING}")

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# These variables must use CACHE, otherwise QtCreator won't see them.
#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android" CACHE INTERNAL "")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        file(GLOB_RECURSE
#            MY_ANDROID_LIBS_LIST
#         /home/johnconn/external_storage/ros2_core_android_ws/install/device/*.so)
#       #message("glob recurse setting to: ")
#
#       foreach(OPT ${MY_ANDROID_LIBS_LIST})
#           set(MY_ANDROID_STRING "${MY_ANDROID_STRING} ${OPT}")
#       endforeach()
#       #message("${MY_ANDROID_STRING}")


#        #list(APPEND ANDROID_EXTRA_LIBS CACHE INTERNAL "")

#        #list(APPEND ANDROID_EXTRA_LIBS  CACHE INTERNAL "")

#    endif()
#endif()
