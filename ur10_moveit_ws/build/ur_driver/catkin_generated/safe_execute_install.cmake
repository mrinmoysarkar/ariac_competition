execute_process(COMMAND "/home/mrinmoy/ariac_competition/ur10_moveit_ws/build/ur_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/mrinmoy/ariac_competition/ur10_moveit_ws/build/ur_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
