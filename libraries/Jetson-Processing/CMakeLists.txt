PROJECT(nvidia C CXX ASM)

set(EIGEN_DIR "/usr/local/Cellar/eigen/3.3.5/include/eigen3/Eigen")
include_directories(${EIGEN_DIR})
target_link_libraries(nvidia ${EIGEN_DIR})
