#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/kobuki_desktop/kobuki_dashboard"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/team_cyber_crusaders/Desktop/Robotki/workspace/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/team_cyber_crusaders/Desktop/Robotki/workspace/install/lib/python3/dist-packages:/home/team_cyber_crusaders/Desktop/Robotki/workspace/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/team_cyber_crusaders/Desktop/Robotki/workspace/build" \
    "/usr/bin/python3" \
    "/home/team_cyber_crusaders/Desktop/Robotki/workspace/src/kobuki_desktop/kobuki_dashboard/setup.py" \
     \
    build --build-base "/home/team_cyber_crusaders/Desktop/Robotki/workspace/build/kobuki_desktop/kobuki_dashboard" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/team_cyber_crusaders/Desktop/Robotki/workspace/install" --install-scripts="/home/team_cyber_crusaders/Desktop/Robotki/workspace/install/bin"
