########## Import Compression Streams dependencies
FIND_PACKAGE(zlib)
if(ZLIB_FOUND)
    message(STATUS "Import ZLIB lib. [${ZLIB_LIBRARIES}]")
    add_definitions(-D_USE_GZSTREAM)
endif()

FIND_PACKAGE(zstd)
if(ZSTD_FOUND)
    if(zstd_LIBRARIES)
        set(ZSTD_LIBRARIES ${zstd_LIBRARIES})
    endif()
    message(STATUS "Import ZSTD lib. [${ZSTD_LIBRARIES}]")
    add_definitions(-D_USE_ZSTDSTREAM)
endif()

SET(InternalSRC ARInterface.cpp
                zstdstream/zstdstream.cpp)
set(InternalHDR ARInterface.h
                ARInterface_impl.hpp)

set(LIBRARY_FILES_C Common.cpp
                    ${InternalSRC}
                    ARReader.cpp
                    ARWriter.cpp
                    )
set(LIBRARY_FILES_H ${InternalHDR}
                    Common.h
                    ARReader.h
                    ARWriter.h)

#cxx_library_with_type_no_pch(MVArchive "Libs" "STATIC" "${cxx_default}"
#	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
#)

add_library(MVArchive ${LIBRARY_FILES_C})
# Manually set Common.h as the precompiled header
#set_target_pch(MVArchive Common.h)

target_link_libraries(MVArchive PUBLIC ${ZLIB_LIBRARIES} ${ZSTD_LIBRARIES} ${OpenCV_LIBRARIES} ${COMPRESSED_STREAMS_LIBRARIES})
target_compile_definitions(MVArchive PUBLIC _USE_OPENCV _USE_GZSTREAM _USE_ZSTDSTREAM OFFICIAL_OPENMVS PRIVATE ARCHIVE_DLL_EXPORTS )

set_target_properties(MVArchive PROPERTIES
        PUBLIC_HEADER "${LIBRARY_FILES_H}")

install(TARGETS MVArchive
        EXPORT OpenMVSTargets
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/MVArchive" COMPONENT dev)