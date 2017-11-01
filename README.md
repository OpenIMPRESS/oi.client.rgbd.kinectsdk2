# oi.client.rgbd.kinectsdk2

Stream KinectV2 data using the Microsoft Kinect SDK through UDP accross the network.

## Installation (TODO: THIS IS NOT TESTED, check https://homes.cs.washington.edu/~edzhang/tutorials/kinect2/kinect0.html )
 - Install Kinect SDK 2.0 (TODO: Explain)
 - Get a copy of asio:
   - git clone https://github.com/chriskohlhoff/asio.git
 - make a folder "build" in oi.client.rgbd.kinectsdk2
   - cd build
   - cmake -G "Visual Studio 14 2015 Win64" -D asio\_ROOT\_DIR="path/to/asio" ..
   - (or your respective string for your version of visual studio)
   - (default locations for search for asio folder is next to this folder)
 - Now you can open oi.client.rgbd.kinectsdk2.sln with Visual Studio
