file(GLOB SRC . *.cpp)
include_directories(/home/wangguojun/source_code/libgp/include)
add_library(extract ${SRC})
target_link_libraries(extract ${PCL_LIBRARIES} ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
