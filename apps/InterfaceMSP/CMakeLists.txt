find_package(spdlog REQUIRED)
find_package(cxxopts REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/LAStools/LASlib/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/LAStools/LASzip/src)

add_subdirectory(LAStools)

include_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(InterfaceMSP main.cpp)
target_link_libraries(InterfaceMSP cxxopts::cxxopts spdlog::spdlog LASlib MVArchive)