execute_process(COMMAND "/home/scott/catkin_ws_tb3_AGV/build/tb3_autonomous_op/tb3_driving/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/scott/catkin_ws_tb3_AGV/build/tb3_autonomous_op/tb3_driving/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
