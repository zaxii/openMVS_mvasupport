find_package(spdlog REQUIRED)
find_package(cxxopts REQUIRED)

add_executable(ImageDeblur main.cpp)
target_link_libraries(ImageDeblur cxxopts::cxxopts spdlog::spdlog ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX)