# Install script for directory: /home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/team_cyber_crusaders/Desktop/Robotki/workspace/install")
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
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/ARPair.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/ARPairList.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/Wall.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/WallList.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/Column.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/ColumnList.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/MagicButton.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/NavigationControl.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/NavigationControlStatus.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/Table.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/TableList.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/Trajectory.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/TrajectoryList.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/Waypoint.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/msg/WaypointList.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/srv" TYPE FILE FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/srv/WaypointListService.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/action" TYPE FILE FILES
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/action/NavigateTo.action"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/action/DockingInteractor.action"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/action/Localize.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/msg" TYPE FILE FILES
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToAction.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToActionGoal.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToActionResult.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToActionFeedback.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToGoal.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToResult.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/NavigateToFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/msg" TYPE FILE FILES
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorAction.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorActionGoal.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorActionResult.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorActionFeedback.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorGoal.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorResult.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/DockingInteractorFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/msg" TYPE FILE FILES
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeAction.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeActionGoal.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeActionResult.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeActionFeedback.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeGoal.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeResult.msg"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/yocs_msgs/msg/LocalizeFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/yocs_msgs" TYPE FILE FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/include/yocs_msgs/JoystickConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/yocs_msgs" TYPE FILE FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/yocs_msgs" TYPE DIRECTORY FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/cmake" TYPE FILE FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/include/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/roseus/ros/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/common-lisp/ros/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/share/gennodejs/ros/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/devel/lib/python3/dist-packages/yocs_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/cmake" TYPE FILE FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs/cmake" TYPE FILE FILES
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgsConfig.cmake"
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/build/yocs_msgs/catkin_generated/installspace/yocs_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yocs_msgs" TYPE FILE FILES "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/yocs_msgs/package.xml")
endif()

