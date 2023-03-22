execute_process(COMMAND "/home/team_cyber_crusaders/Desktop/Robotki/workspace/build/kobuki_desktop/kobuki_dashboard/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/team_cyber_crusaders/Desktop/Robotki/workspace/build/kobuki_desktop/kobuki_dashboard/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
