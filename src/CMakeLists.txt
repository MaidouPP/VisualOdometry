add_library(vslam
orb.cpp vo.cpp
)
target_link_libraries( vslam
  ${OpenCV_LIBS}
)

add_executable( featureExtract featureExtract.cpp)
target_link_libraries(featureExtract
    vslam
    ${OpenCV_LIBS}
)

add_executable( vo vo_test.cpp)
target_link_libraries(vo
    vslam
    ${OpenCV_LIBS}
)
