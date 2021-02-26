**Installation**

## Requirements

1. Installe the BehaviorTree.CPP library from https://gitlab.com/aromerogarces/behaviortree.git

2. Install RealSense and OpenVino using the installer script available in https://gitlab.com/aromerogarces/installers-sa3ir

3. AprilTags from: https://github.com/AprilRobotics/apriltag

4. Make sure you have installed the following packages from the Ubuntu repository:

    sudo apt-get install libgflags-dev libopencv-dev libcpprest-dev libjsoncpp-dev

## Installation itself

*cd* to $ROBOCOMP/components directory  and type:

    git clone https://gitlab.com/grupo-avispa/sa3ir.git


Let's compile the whole thing:

    cd sa3ir
    mkdir build
    cd build
    cmake ..
    make

## Testing Installation

Start yakuake terminal emulator

*cd* to $ROBOCOMP/components/sa3ir

Let's run the use case:

./start.sh

When finished, run the following:

./kill.sh

and close yakuake tabs using (CTRL + Shift + r)
