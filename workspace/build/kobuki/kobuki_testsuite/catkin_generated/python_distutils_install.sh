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

echo_and_run cd " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/kobuki/kobuki_testsuite"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH=" ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/install/lib/python3/dist-packages: ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR=" ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build" \
    "/usr/bin/python3" \
    " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/src/kobuki/kobuki_testsuite/setup.py" \
     \
    build --build-base " ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/build/kobuki/kobuki_testsuite" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix=" ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/install" --install-scripts=" ~/Documents/FRI/3-letnik/RINS/DN3/Robotki/workspace/install/bin"
