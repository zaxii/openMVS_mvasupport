include(ExternalProject)

ExternalProject_Add(zstd_ext
    URL                 https://github.com/facebook/zstd/archive/v1.3.8.tar.gz
    PREFIX              zstd
    CMAKE_ARGS          -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                        -DZSTD_BUILD_PROGRAMS=OFF 
                        -DZSTD_BUILD_SHARED=OFF 
                        -DZSTD_LEGACY_SUPPORT=OFF
                        -DCMAKE_POSITION_INDEPENDENT_CODE=${CMAKE_POSITION_INDEPENDENT_CODE}
                        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
                        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
                        -DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}
                        -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    CMAKE_GENERATOR     ${CMAKE_GENERATOR}
    CMAKE_GENERATOR_PLATFORM ${CMAKE_GENERATOR_PLATFORM}
    PATCH_COMMAND       ${CMAKE_COMMAND}
                          -E copy ${CMAKE_SOURCE_DIR}/build/Modules/patch/zstd_CMakeLists.txt CMakeLists.txt
    INSTALL_COMMAND     ""
)

ExternalProject_Get_Property(zstd_ext SOURCE_DIR BINARY_DIR)

add_library(zstd STATIC IMPORTED GLOBAL)
add_dependencies(zstd zstd_ext)

if(MSVC)
    set_target_properties(zstd
        PROPERTIES
            IMPORTED_LOCATION ${BINARY_DIR}/build/cmake/lib/zstd_static.lib
    )
else()
    set_target_properties(zstd
        PROPERTIES
            IMPORTED_LOCATION ${BINARY_DIR}/build/cmake/lib/libzstd.a
    )
endif()

set(ZSTD_FOUND TRUE)
set(ZSTD_INCLUDE_DIR ${SOURCE_DIR}/lib)
set(ZSTD_LIBRARY zstd)
set(ZSTD_LIBRARIES zstd)
