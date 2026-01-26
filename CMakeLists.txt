cmake_minimum_required(VERSION 3.16)

project(LIDARMapping_HMI VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Enforce C++23 support
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# set(CMAKE_PREFIX_PATH "/opt/Qt/6.10.1/gcc_64")
set(Qt6_DIR "/opt/Qt/6.10.1/gcc_64/lib/cmake/Qt6")



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

find_package(Qt6 6.8 REQUIRED COMPONENTS Quick QuickControls2)


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
    find_package(std_msgs REQUIRED)
endif()

if(USE_QT)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

    file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h *.hpp)
    file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp *.c)
    file(GLOB QML_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ui/*.qml)
    file(GLOB RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.qrc)

    qt_standard_project_setup(REQUIRES 6.8)

    qt_add_executable(${PROJECT_NAME}_App
       ${HEADERS}
       ${SOURCES}
       ${QML_FILES}
       ${RESOURCES}
    )

    qt_add_qml_module(${PROJECT_NAME}_App
       URI ${PROJECT_NAME}
       QML_FILES ui/Main.qml
       QML_FILES ui/PilotLight.qml
       QML_FILES ui/DirectionButton.qml
       SOURCES include/MainBackendHelper.h src/MainBackendHelper.cpp
    )

    target_include_directories(${PROJECT_NAME}_App PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    if(USE_ROS)
        target_link_libraries(${PROJECT_NAME}_App
            PRIVATE rclcpp::rclcpp ${std_msgs_TARGETS}
        )
    endif()

    target_link_libraries(${PROJECT_NAME}_App
        PRIVATE Qt6::Quick
     )


    if(WIN32)
        set_target_properties(${PROJECT_NAME}_App PROPERTIES WIN32_EXECUTABLE TRUE)
    endif()

    if(APPLE)
        set_target_properties(${PROJECT_NAME}_App PROPERTIES
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_GUI_IDENTIFIER com.example.${PROJECT_NAME}
            MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
            MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )
    endif()
endif()

include(GNUInstallDirs)
#set(ENV{LD_LIBRARY_PATH} "$ENV{LD_LIBRARY_PATH}:/opt/Qt/6.10.1/gcc_64/lib")

if(USE_ROS)
    install(TARGETS ${PROJECT_NAME}_App
        DESTINATION lib/${PROJECT_NAME}_App
    )

    install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}_App
    )

    ament_package()
else()
    if(USE_QT)
        install(TARGETS ${PROJECT_NAME}_App
            BUNDLE DESTINATION .
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )
    else()
        message(WARNING "Neither ROS nor Qt support available. No targets will be built.")
    endif()
endif()

if(USE_ROS)
    target_compile_definitions(${PROJECT_NAME}_App PRIVATE USE_ROS)
endif()
