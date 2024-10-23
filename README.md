# android-franka-client
Several Android apps to control the franka panda robotic arm. The repo is archived because all related projects are finished/suspended. The demo for some of which could be found in https://github.com/BobbyMa2014/ar-robot-control-demo

## franka-control-client
An app to move the robotic arm to a certain desired position. The user firstly gives input of 7 joint angles, then it automatically encodes the angle into a certain format and send the data to the robotic arm (server), whose IP and port number are specified.

## franka-ik-portable
An app to calculate the inverse kinematics (IK) of robotic arm. The original IK uses C++ and runs on a server. However, by integrating CMake the same function could be achieved on a portable application. Uses the "Eigen" library.

Basically it works as a "bridge" between the client and server. The client firstly sends coordinate data (xyz) to the smartphone. The phone then solves the IK in real time and sends the joint angles to the robotic arm (server).

## lab-1
A lab demonstration to allow the robotic arm install/uninstall circuit components at fixed position. Used to demonstrate resistors.


## lab-2
A second lab demonstration to allow the robotic arm install/uninstall circuit components at fixed position. Used to demonstrate filters.
