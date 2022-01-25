################
## Conan.io
################
# Download automatically, you can also just copy the conan.cmake file
if (NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
    message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
    file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/master/conan.cmake"
            "${CMAKE_BINARY_DIR}/conan.cmake")
endif ()

include(${CMAKE_BINARY_DIR}/conan.cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

set(CONAN_CMAKE_OPTIONS "")
if(UNIX)
    set(CONAN_CMAKE_OPTIONS "opencv:with_gtk=False ${CONAN_CMAKE_OPTIONS}")
endif()
conan_cmake_configure(
        REQUIRES
        eigen/3.3.9
        boost/1.75.0
        libcurl/7.77.0
        zstd/1.5.0
        zlib/1.2.11
        opencv/4.5.3
        gmp/6.2.1
        mpfr/4.1.0
        cgal/5.3
        ceres-solver/2.0.0
#        openmesh/8.1
        fmt/8.0.1
        spdlog/1.9.2
        GENERATORS cmake_find_package
        #	IMPORTS "bin, *.dll.* -> ./bin"
        #	IMPORTS "lib, *.dylib.* -> ./bin"
        #	IMPORTS "lib, *.so.* -> ./bin"
        OPTIONS ${CONAN_CMAKE_OPTIONS}
)

conan_cmake_autodetect(settings
        BUILD_TYPE "Release")

conan_cmake_install(PATH_OR_REFERENCE .
        BUILD missing
        REMOTE conancenter
        SETTINGS ${settings})

FIND_PACKAGE(OpenCV)
if(OpenCV_FOUND)
    set(OpenCV_INCLUDE_DIRS ${opencv_INCLUDE_DIRS})
    set(OpenCV_INCLUDE_DIR ${opencv_INCLUDE_DIR})
    set(OpenCV_INCLUDES ${opencv_INCLUDES})
    set(OpenCV_DEFINITIONS ${opencv_DEFINITIONS})
    set(OpenCV_LIBRARIES ${opencv_LIBRARIES})
    set(OpenCV_LIBS ${opencv_LIBS})
    set(OpenCV_LIBRARIES_TARGETS ${opencv_LIBRARIES_TARGETS})
endif()
#
#FIND_PACKAGE(gmp)
#if(gmp_FOUND)
#    INCLUDE_DIRECTORIES(${gmp_INCLUDE_DIRS})
#    ADD_DEFINITIONS(${gmp_DEFINITIONS})
#endif()
#
#FIND_PACKAGE(mpfr)
#if(mpfr_FOUND)
#    INCLUDE_DIRECTORIES(${mpfr_INCLUDE_DIRS})
#    ADD_DEFINITIONS(${mpfr_DEFINITIONS})
#endif()