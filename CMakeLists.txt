cmake_minimum_required (VERSION 3.16)
project (dvrk_camera_registration VERSION 1.1.0)

# ROS 1
find_package (
  catkin QUIET
  COMPONENTS rospy
  )

if (catkin_FOUND)

  catkin_python_setup ()
  catkin_package ()

else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)

  if (ament_cmake_FOUND)
    find_package (ament_cmake_python REQUIRED)
    ament_python_install_package (dvrk_camera_registration
      PACKAGE_DIR ${dvrk_camera_registration_SOURCE_DIR}/src/dvrk_camera_registration)
    file (GLOB dvrk_SCRIPTS scripts/*.py)
    install (PROGRAMS ${dvrk_SCRIPTS}
      DESTINATION lib/${PROJECT_NAME})
    ament_package ()
  endif (ament_cmake_FOUND)

endif (catkin_FOUND)
