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


## Command Line Parameters

-mm [1|0]  

Use matchmaking (1=yes, 0=configure ip/port manually)

-id [socketID]  

The socket name.

-lp [portnumber]  

The port on which this streamer is listening for commands. Automatically set if using matchmaking.

-rp [portnumber]  

The port on the streaming destination to which data is sent. If using matchmaking, this is the port of the matchmaking  server.

-rh [hostname/ip]  

The ip/hostname on the streaming destination to which data is sent. If using matchmaking, this is the ip/hostname of the matchmaking  server.

-sn [serialnumber]  

The serial number of the kinect.

-pp [pipeline]  

Not used for microsoft sdk.

-md [???]  
Max depth. Don't use this.

-o [filename]  
Default file to store recordings to.

-i [filename]  
Default file to load recordings from

-d [folder path]  
Folder to load/store recordings in/from.

-dl [0-10]  
Debug level


## Minimal Example: Stream Kinect to a local unity client
Run the binary from CMD:  

 ./oi.client.rgbd.kinectsdk2.exe -rh 127.0.0.1 -rp 5001 -lp 5005 -mm 0  
 
Use the "LocalExample" prefab from oi.plugin.rgbd to render this pointcloud.
