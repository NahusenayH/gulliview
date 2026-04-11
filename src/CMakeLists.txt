
add_executable(GulliView GulliView/Main.cpp GulliView/AccelerationTracker.cpp GulliView/AngleTracker.cpp GulliView/CalibrateCameras.cpp GulliView/DebugLogger.cpp GulliView/FastSearch.cpp GulliView/FastSearchFunctions.cpp GulliView/FastThread.cpp GulliView/GeneralSearchFunctions.cpp GulliView/GlobalCoordination.cpp GulliView/GUI.cpp GulliView/InitCameras.cpp GulliView/LogTime.cpp GulliView/NiceThread.cpp GulliView/ProcessCamera.cpp GulliView/ProducerThread.cpp GulliView/TransformFrame.cpp GulliView/Undistortion.cpp CameraUtil.h CameraUtil.cpp TagFamily.cpp TagFamily.h)
target_link_libraries(GulliView ${OPENCV_LDFLAGS})
target_link_libraries(GulliView ${Boost_LIBRARIES})
target_link_libraries(GulliView ${APRILTAGS_LDFLAGS})

add_executable(Transmitter Transmitter.cpp)
target_link_libraries(Transmitter ${OPENCV_LDFLAGS})
target_link_libraries(Transmitter ${Boost_LIBRARIES})
target_link_libraries(Transmitter ${APRILTAGS_LDFLAGS})

if (GLUT_LIBRARY)

  #add_executable(gltest gltest.cpp)
  #target_link_libraries(gltest ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${AT_LIBS})

endif()
