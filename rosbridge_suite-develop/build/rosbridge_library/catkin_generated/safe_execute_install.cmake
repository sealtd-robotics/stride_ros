execute_process(COMMAND "/home/chedanix/work/stride/ros/rosbridge_suite-develop/build/rosbridge_library/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/chedanix/work/stride/ros/rosbridge_suite-develop/build/rosbridge_library/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
