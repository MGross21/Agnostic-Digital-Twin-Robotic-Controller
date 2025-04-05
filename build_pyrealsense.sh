#!/bin/bash




CURRENT_DIR=$(pwd)

# Install prerequisites
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y cmake g++ python3-dev libx11-dev xorg-dev \
libvulkan-dev pkg-config libudev-dev libgl1-mesa-dev libglu1-mesa-dev \
freeglut3-dev, libusb-1.0-0-dev


# Clone the librealsense repository
if [ -d "librealsense" ]; then
    if [ -d "librealsense/.git" ]; then
        echo "librealsense directory already exists. Pulling latest changes..."
        cd librealsense
        git pull
        cd ..
    else
        echo "librealsense directory exists but is not a valid Git repository. Removing it..."
        rm -rf librealsense
        echo "Cloning librealsense repository..."
        git clone https://github.com/IntelRealSense/librealsense.git
    fi
else
    echo "Cloning librealsense repository..."
    git clone https://github.com/IntelRealSense/librealsense.git
fi

# Navigate to the librealsense directory
cd librealsense

if [ -d "build" ]; then
    echo "Cleaning up previous build directory..."
    rm -rf build
fi

# Create a build directory
mkdir build && cd build

# Configure with CMake
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_WITH_VULKAN=OFF -DUSE_X11=OFF -DBUILD_WITH_OPENGL=OFF

# Build the project
make -j4

# Install the pyrealsense2 wrapper
sudo make install

# Copy the .so files to your project's directory
cd ../wrappers/python
ls $CURRENT_DIR/pyrealsense2*.so # List the .so files
cp pyrealsense2*.so $CURRENT_DIR

cd $CURRENT_DIR