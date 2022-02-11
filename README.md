**Installation**

## Requirements

1. Install the BehaviorTree.CPP library from https://gitlab.com/aromerogarces/behaviortree.git

2. Install RealSense (https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

3. Install OpenVino from https://github.com/openvinotoolkit/openvino

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

Installing node modules:

    cd node
    npm install

## Testing Installation

Start yakuake terminal emulator

*cd* to $ROBOCOMP/components/sa3ir

Start the web server:

nodejs server.js

Let's run the use case:

./start.sh

When finished, run the following:

./kill.sh

and close yakuake tabs using (CTRL + Shift + r)
