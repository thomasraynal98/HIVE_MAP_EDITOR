CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(HIVE_MAP_EDITOR VERSION 1.0)

# include directories.
INCLUDE_DIRECTORIES(include)

# set good gcc version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -L/usr/local/lib")

# found package.
FIND_PACKAGE(OpenCV 4.2 REQUIRED)
FIND_PACKAGE(OpenXLSX REQUIRED)

# add the executable exemple.
ADD_EXECUTABLE(MAP_VISUALISATOR 
    software/MAP_VISUALISATOR.cpp
    src/useful.cpp
    include/useful.h)

TARGET_LINK_LIBRARIES(MAP_VISUALISATOR
    pthread
    OpenXLSX::OpenXLSX
    ${OpenCV_LIBS})

# add the executable exemple.
ADD_EXECUTABLE(HIVE_MAP_EDITOR 
    software/HIVE_MAP_EDITOR.cpp
    src/useful.cpp
    include/useful.h)

TARGET_LINK_LIBRARIES(HIVE_MAP_EDITOR
    pthread
    OpenXLSX::OpenXLSX
    ${OpenCV_LIBS})

# ping code.
ADD_EXECUTABLE(PING_CODE 
    software/66_exemple.cpp)

TARGET_LINK_LIBRARIES(PING_CODE
    ssl
    crypto 
    /usr/local/lib/libsioclient_tls.so
    pthread)