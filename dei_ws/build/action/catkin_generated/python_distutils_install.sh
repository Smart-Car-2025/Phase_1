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

echo_and_run cd "/home/smark/bfmc_2022/dei_ws/src/action"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/smark/bfmc_2022/dei_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/smark/bfmc_2022/dei_ws/install/lib/python2.7/dist-packages:/home/smark/bfmc_2022/dei_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/smark/bfmc_2022/dei_ws/build" \
    "/usr/bin/python2" \
    "/home/smark/bfmc_2022/dei_ws/src/action/setup.py" \
     \
    build --build-base "/home/smark/bfmc_2022/dei_ws/build/action" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/smark/bfmc_2022/dei_ws/install" --install-scripts="/home/smark/bfmc_2022/dei_ws/install/bin"
