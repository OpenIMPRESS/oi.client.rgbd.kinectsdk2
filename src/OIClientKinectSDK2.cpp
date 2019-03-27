/*
This file is part of the OpenIMPRESS project.

OpenIMPRESS is free software: you can redistribute it and/or modify
it under the terms of the Lesser GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

OpenIMPRESS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with OpenIMPRESS. If not, see <https://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <cstdlib>
#include <signal.h>

#include <RGBDStreamer.hpp>
#include <KinectSDK2DeviceInterface.hpp>

using namespace oi::core::rgbd;
using namespace oi::client::kinectsdk2;

asio::io_service io_service;

int main(int argc, char *argv[]) {
    RGBDStreamerConfig config(argc, argv);
    KinectSDK2DeviceInterface device(config.deviceSerial);
    RGBDStreamer streamer(device, config, io_service);
    
    while (true) {
        device.Cycle(&streamer);
    }

	device.CloseDevice();
	// close streamer...
}
