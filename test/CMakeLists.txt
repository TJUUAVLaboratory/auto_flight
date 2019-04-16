# add_executable(config_test config_test.cpp)
# target_link_libraries(config_test Flight)

add_executable(mavros_flying_test mavros_flying_test.cpp)
target_link_libraries(mavros_flying_test Flight ${catkin_LIBRARIES})

add_executable(airsim_flying_test airsim_flying_test.cpp)
target_link_libraries(airsim_flying_test Flight ${catkin_LIBRARIES})

add_executable(flying_pose_calibration flying_pose_calibration.cpp)
target_link_libraries(flying_pose_calibration Flight ${catkin_LIBRARIES})

add_executable(vo_estimate_relay vo_estimate_relay.cpp)
target_link_libraries(vo_estimate_relay Flight ${catkin_LIBRARIES})

add_executable(octomap_build_test octomap_build_test.cpp)
target_link_libraries(octomap_build_test Flight ${catkin_LIBRARIES})


add_executable(offline_pclmap_test offline_pclmap_test.cpp)
target_link_libraries(offline_pclmap_test Flight ${catkin_LIBRARIES})



### 
add_executable(setting_destination set_destination.cpp)
target_link_libraries(setting_destination ${catkin_LIBRARIES})
###

### 
add_executable(mavros_target_position mavros_target_position.cpp)
target_link_libraries(mavros_target_position ${catkin_LIBRARIES})
###

add_executable(trajplan_test trajplan_test.cpp)
target_link_libraries(trajplan_test ${AIRSIM_LIBS} Flight  ${catkin_LIBRARIES})

#  add_executable(drone_test
#   drone_test.cpp
# )
# target_link_libraries(drone_test
#   ${AIRSIM_LIBS}
#   Flight 
#   ${catkin_LIBRARIES} 
#   )

   add_executable(airsim_get_image
  airsim_get_image.cpp
)
target_link_libraries(airsim_get_image
  ${AIRSIM_LIBS}
  Flight 
  ${catkin_LIBRARIES} 
  )

# add_executable(enu_neu_frame2
#  enu_ned_frame2.cpp
# )
# target_link_libraries(enu_neu_frame2
#   ${catkin_LIBRARIES}
#   ${AIRSIM_LIBS}
# )