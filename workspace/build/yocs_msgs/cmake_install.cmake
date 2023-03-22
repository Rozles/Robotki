# Install script for directory:  ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/msg" TYPE FILE FILES
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/ARPair.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/ARPairList.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/Wall.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/WallList.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/Column.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/ColumnList.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/MagicButton.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/NavigationControl.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/NavigationControlStatus.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/Table.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/TableList.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/Trajectory.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/TrajectoryList.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/Waypoint.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/msg/WaypointList.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/srv" TYPE FILE FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/srv/WaypointListService.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/action" TYPE FILE FILES
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/action/NavigateTo.action"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/action/DockingInteractor.action"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/action/Localize.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/msg" TYPE FILE FILES
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToAction.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToActionGoal.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToActionResult.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToActionFeedback.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToGoal.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToResult.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/msg" TYPE FILE FILES
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorAction.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorActionGoal.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorActionResult.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorActionFeedback.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorGoal.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorResult.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/msg" TYPE FILE FILES
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeAction.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeActionGoal.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeActionResult.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeActionFeedback.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeGoal.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeResult.msg"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/yocs_msgs" TYPE FILE FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/include/yocs_msgs/JoystickConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/yocs_msgs" TYPE FILE FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/yocs_msgs" TYPE DIRECTORY FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/cmake" TYPE FILE FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/include/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/roseus/ros/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/common-lisp/ros/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/share/gennodejs/ros/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/cmake" TYPE FILE FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/cmake" TYPE FILE FILES
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgsConfig.cmake"
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs" TYPE FILE FILES " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/yocs_msgs/package.xml")
endif()

