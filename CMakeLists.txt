##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME alphanumeric_viewer)
project(${PROJECT_NAME} C CXX)

add_definitions(-std=c++11)
##############################################################################
# Catkin
##############################################################################
set(ALPHANUMERIC_VIEWER_CONTROLLER_SOURCE_DIR
        src/source)

set(ALPHANUMERIC_VIEWER_CONTROLLER_INCLUDE_DIR
        src/include)


set(ALPHANUMERIC_VIEWER_HEADER_FILES
        ${ALPHANUMERIC_VIEWER_CONTROLLER_INCLUDE_DIR}/*.h
)
set(ALPHANUMERIC_VIEWER_CONTROLLER_TEST_DIR
 src/test
)

set(ALPHANUMERIC_VIEWER_SOURCE_FILES
       src/source/alphanumeric_viewer_process_main.cpp

)


set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs aerostack_msgs lib_cvgutils geometry_msgs  image_transport cv_bridge )
catkin_package(
        DEPENDS OpenCV
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs aerostack_msgs geometry_msgs lib_cvgutils image_transport cv_bridge 
  )
include_directories(${ALPHANUMERIC_VIEWER_INCLUDE_DIR})
include_directories(${ALPHANUMERIC_VIEWER_CONTROLLER_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# check required dependencies
find_package(Boost REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

##############################################################################
# Qwt library
##############################################################################

FIND_PATH(QWT_INCLUDE_DIR NAMES qwt.h PATHS
  /usr/include
  PATH_SUFFIXES qwt
)

MESSAGE(STATUS "qwt header: ${QWT_INCLUDE_DIR}")

find_library(QWT_LIB NAMES qwt-qt5)

MESSAGE(STATUS "Found Qwt: ${QWT_LIB}")

#set(QWT_LIBRARY "/usr/lib/libqwt-qt5.so.6.1.2")


include_directories(${QWT_INCLUDE_DIR})
link_directories(${QWT_LIBRARY})

##############################################################################
# Qt Environment
##############################################################################

# included via the dependency call in package.xml
find_package(Qt5 COMPONENTS Core Gui Svg OpenGL Widgets PrintSupport REQUIRED)
#include(${QT_USE_FILE})

## to avoid conflict with boost signals it is needed to define QT_NO_KEYWORDS.
add_definitions(-DQT_NO_KEYWORDS)
ADD_DEFINITIONS(${QT_DEFINITIONS})

##############################################################################
# Sections
##############################################################################

file(GLOB_RECURSE QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/view/resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS  src/controller/*.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/controller/*.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC")

##############################################################################
# Binaries
##############################################################################


#MESSAGE(STATUS "Found Qt:  ${QT_LIBRARIES} ")
#add_executable(alphanumeric_viewer ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
#add_dependencies(alphanumeric_viewer ${catkin_EXPORTED_TARGETS})
#target_link_libraries(alphanumeric_viewer ${QWT_LIBRARY} ${QT_LIBRARIES} ${catkin_LIBRARIES} )
#install(TARGETS alphanumeric_viewer RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(alphanumeric_viewer_lib ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_MOC_HPP} ${QT_FORMS_HPP})
add_dependencies(alphanumeric_viewer_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(alphanumeric_viewer_lib ${QWT_LIB} ${catkin_LIBRARIES})


add_executable(alphanumeric_viewer ${ALPHANUMERIC_VIEWER_SOURCE_FILES})
add_dependencies(alphanumeric_viewer ${catkin_EXPORTED_TARGETS})
target_link_libraries(alphanumeric_viewer alphanumeric_viewer_lib)
target_link_libraries(alphanumeric_viewer ncurses)
target_link_libraries(alphanumeric_viewer ${OpenCV_LIBS})
target_link_libraries(alphanumeric_viewer ${catkin_LIBRARIES})

