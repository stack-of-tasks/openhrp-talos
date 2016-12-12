# Launch Model Loader corba server
rm /tmp/openhrp-model-loader.log
/opt/ros/indigo/bin/openhrp-model-loader &> /tmp/openhrp-model-loader.log &

# Launch Collision detector corba server
rm /tmp/openhrp-collision-detector.log
/opt/ros/indigo/bin/openhrp-collision-detector &> /tmp/openhrp-collision-detector.log &

# Launch Dynamics simulator corba server
rm /tmp/openhrp-aist-dynamics-simulator.log
/opt/ros/indigo/bin/openhrp-aist-dynamics-simulator &> /tmp/openhrp-aist-dynamics-simulator.log &

# Launch Online viewer corba server
rm /tmp/gepetto-online-viewer.log
/opt/ros/indigo/bin/hrpsys-viewer &> /tmp/hrpsys-online-viewer.log &


# Launch SoT-controller corba server
#/home/ostasse/devel/test/install/bin/controller-talos /opt/openrobots/lib/libsot-pyren-controller.so /home/ostasse/devel/test/install/example/openhrp3-pyren/scheduler/etc/PIDgains.dat &
/bin/sleep 2

# Launch Scheduler of the simulation
/home/ostasse/devel/test/install/example/openhrp3-simulator-wo-rtm/scheduler/schedulerproject  -url /home/ostasse/devel-src/Talos/openhrp-talos/project/test.xml


