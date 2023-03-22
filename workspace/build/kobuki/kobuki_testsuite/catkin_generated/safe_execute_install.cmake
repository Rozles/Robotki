execute_process(COMMAND " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/kobuki/kobuki_testsuite/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process( ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/kobuki/kobuki_testsuite/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
