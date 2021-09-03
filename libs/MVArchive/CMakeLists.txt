########## Import Compression Streams dependencies
option(WITH_COMPRESS_STREAMS "Use Compress Streams to support brotli, zstd, lz4, lzma. [Only works for small size]" OFF)

if(WITH_COMPRESS_STREAMS)
    include(import_compressed_streams)
else()
    include(import_zstd)
    include(import_zlib)
endif()

if(COMPRESSED_STREAMS_FOUND)
    message(STATUS "Import compressed_streams. [${COMPRESSED_STREAMS_LIBRARIES}]")
    add_definitions(-D_USE_COMPRESSED_STREAMS)
    add_definitions(-D_USE_ZSTDSTREAM)
    add_definitions(-D_USE_GZSTREAM)
endif()

if(ZSTD_FOUND)
    message(STATUS "Import ZSTD lib. [${ZSTD_LIBRARIES}]")
    add_definitions(-D_USE_ZSTDSTREAM)
endif()

if(ZLIB_FOUND)
    message(STATUS "Import ZLIB lib. [${ZLIB_LIBRARIES}]")
    add_definitions(-D_USE_GZSTREAM)
endif()

SET(SRCS ARInterface.cpp
        zstdstream/zstdstream.cpp)
set(HEADERS ARInterface.h
        ARInterface_impl.hpp)

add_library(MVArchive STATIC ${SRCS})
target_link_libraries(MVArchive PUBLIC ${ZLIB_LIBRARIES} ${ZSTD_LIBRARIES} ${OpenCV_LIBRARIES} ${COMPRESSED_STREAMS_LIBRARIES})
target_compile_definitions(MVArchive PUBLIC _USE_OPENCV _USE_GZSTREAM _USE_ZSTDSTREAM OFFICIAL_OPENMVS PRIVATE ARCHIVE_DLL_EXPORTS )

set_target_properties(MVArchive PROPERTIES
        PUBLIC_HEADER "${HEADERS}")

install(TARGETS MVArchive
        EXPORT OpenMVSTargets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/MVArchive" COMPONENT dev)