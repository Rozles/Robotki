execute_process(COMMAND " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yujin_ocs/yocs_localization_manager/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process( ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yujin_ocs/yocs_localization_manager/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
