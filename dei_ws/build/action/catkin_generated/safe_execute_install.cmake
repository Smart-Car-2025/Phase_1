execute_process(COMMAND "/home/smark/bfmc_2022/dei_ws/build/action/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/smark/bfmc_2022/dei_ws/build/action/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
