cmake_minimum_required(VERSION 3.16)

project(LIDARMapping VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Enforce C++23 support

find_package(Qt6 6.8 REQUIRED COMPONENTS Quick QuickControls2)

qt_standard_project_setup(REQUIRES 6.8)

qt_add_executable(${PROJECT_NAME}App
    main.cpp
)

qt_add_qml_module(${PROJECT_NAME}App
    URI ${PROJECT_NAME}
    QML_FILES Main.qml
    QML_FILES PilotLight.qml
    QML_FILES DirectionButton.qml
    QML_FILES
    SOURCES MainBackendHelper.h MainBackendHelper.cpp
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(${PROJECT_NAME}App PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appLIDARMapping
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(${PROJECT_NAME}App
    PRIVATE Qt6::Quick
)

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}App
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
