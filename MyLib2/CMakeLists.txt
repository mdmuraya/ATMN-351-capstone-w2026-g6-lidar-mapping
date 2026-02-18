cmake_minimum_required(VERSION 3.16)

project(MyLib2 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)

add_library(MyLib2 SHARED
  MyLib2_global.h
  mylib2.cpp
  mylib2.hpp
)

target_link_libraries(MyLib2 PRIVATE Qt${QT_VERSION_MAJOR}::Core)

target_compile_definitions(MyLib2 PRIVATE MYLIB2_LIBRARY)
