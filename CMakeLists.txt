# /home/atmn-351/Command to run the LIDAR_Mapping_HMI_App.txt
# /home/atmn-351/ROS2_Workspaces/LIDAR_Mapping_Workspace/src/LIDAR_Mapping/build/Desktop_Qt_6_10_2-Debug/LIDAR_Mapping_HMI&

# ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=192.168.50.101 udp_receiver_ip:=192.168.50.105
# pointcloud2 topic = /cloud_unstructured_fullframe

cmake_minimum_required(VERSION 4.2)

project(LIDAR_Mapping VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Enforce C++23 support
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    message(STATUS "Target architecture is x64")
    # Add x64 specific code here
    set(Qt6_DIR "/opt/Qt/6.10.2/gcc_64/lib/cmake/Qt6")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "Target architecture is ARM64")
    # Add ARM64 specific code here
    set(Qt6_DIR "/opt/Qt/6.10.2/gcc_arm64/lib/cmake/Qt6")
else()
    message(STATUS "Target architecture is unknown or not explicitly handled: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Define the paths to the external library files
set(EXTERNAL_LIB_DIR "lib")
set(EXTERNAL_INC_DIR "lib/include")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(USE_ROS "Build with ROS2 support" ON)

if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "ROS environment detected: $ENV{ROS_DISTRO}")
    set(USE_ROS ON)
else()
    message(STATUS "ROS environment not detected. Building without ROS support.")
    set(USE_ROS OFF)
endif()

find_package(Qt6 6.8 REQUIRED COMPONENTS Quick QuickControls2 Quick3D Concurrent)

# If you are using PCL directly in C++, you may also need: find_package(MPI REQUIRED)
#find_package(VTK REQUIRED)
#find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(Qt6_FOUND)
    message(STATUS "Qt6 found. Building with Qt support.")
    set(USE_QT ON)
else()
    message(WARNING "Qt6 not found. Building without Qt support.")
    set(USE_QT OFF)
endif()

if(USE_ROS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(example_interfaces REQUIRED)
    find_package(laser_geometry REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)
    #find_package(pcl_conversions REQUIRED)
    #find_package(pcl_ros REQUIRED)

endif()

if(USE_QT)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

    file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
    file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
    file(GLOB QML_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.qml)
    file(GLOB RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.qrc)

    qt_standard_project_setup(REQUIRES 6.8)

    qt_add_executable(${PROJECT_NAME}_HMI
       ${HEADERS}
       ${SOURCES}
       ${QML_FILES}
       ${RESOURCES}
    )

    qt_add_qml_module(${PROJECT_NAME}_HMI
       URI ${PROJECT_NAME}
       QML_FILES ui/HMI.qml
       QML_FILES ui/PilotLight.qml
       QML_FILES ui/SafetyIndicator.qml
       QML_FILES ui/DirectionButton.qml
       SOURCES include/HMIBackendHelper.hpp src/HMIBackendHelper.cpp
    )
    add_subdirectory(lib/libplctag)
    add_library(libplctag STATIC IMPORTED)    

    set_target_properties(libplctag PROPERTIES
        IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libplctag/bin/libplctag_static.a"
    )

    target_include_directories(${PROJECT_NAME}_HMI PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    if(USE_ROS)
        target_link_libraries(${PROJECT_NAME}_HMI
            PRIVATE
                rclcpp::rclcpp
                ${example_interfaces_TARGETS}
                ${laser_geometry_TARGETS}
                ${sensor_msgs_TARGETS}
                ${tf2_ros_TARGETS}
                ${pcl_conversions_TARGETS}
                ${pcl_ros_TARGETS}
        )        
    endif()

    target_link_libraries(${PROJECT_NAME}_HMI
        PRIVATE Qt6::Quick Qt6::Quick3D Qt6::Concurrent  libplctag #MPI::MPI_C ${PCL_LIBRARIES}
    )

    if(WIN32)
        set_target_properties(${PROJECT_NAME}_HMI PROPERTIES WIN32_EXECUTABLE TRUE)
    endif()

    if(APPLE)
        set_target_properties(${PROJECT_NAME}_HMI PROPERTIES
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_GUI_IDENTIFIER com.example.${PROJECT_NAME}
            MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
            MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )
    endif()
endif()

include(GNUInstallDirs)

if(USE_ROS)
    install(TARGETS ${PROJECT_NAME}_HMI
        DESTINATION lib/${PROJECT_NAME}_HMI
    )

    install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}_HMI
    )

    ament_package()
else()
    if(USE_QT)
        install(TARGETS ${PROJECT_NAME}_HMI
            BUNDLE DESTINATION .
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    else()
        message(WARNING "Neither ROS nor Qt support available. No targets will be built.")
    endif()
endif()

if(USE_ROS)
    target_compile_definitions(${PROJECT_NAME}_HMI PRIVATE USE_ROS)
endif()

add_subdirectory(lib/MyLib3)

