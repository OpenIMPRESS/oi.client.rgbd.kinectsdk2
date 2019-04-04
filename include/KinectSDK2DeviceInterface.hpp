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
#pragma once

#include <RGBDDevice.hpp>
#include <Kinect.h>

using namespace oi::core;

namespace oi { namespace client { namespace kinectsdk2 {

template<class Interface>
inline void SafeRelease(Interface *& IRelease);

class KinectSDK2DeviceInterface : public oi::core::rgbd::RGBDDeviceInterface {
public:
    KinectSDK2DeviceInterface(std::string serial);
    int OpenDevice();
    int CloseDevice();
    int Cycle(oi::core::rgbd::RGBDDevice * streamer);
    
    int frame_width();
    int frame_height();
    int send_depth_stride();
    int raw_depth_stride();
    int raw_color_stride();
    
    float device_cx();
    float device_cy();
    float device_fx();
    float device_fy();
    float device_depth_scale();
    
    bool supports_audio();
    bool supports_body();
    bool supports_bidx();
    bool supports_hd();
    
    std::string device_guid();
    TJPF color_pixel_format();


	int color_width();
	int color_height();

	// 0 for pending, 1 for good, -X for error
	int GetFrame(IMultiSourceFrame *, IColorFrame **);
	int GetFrame(IMultiSourceFrame *, IDepthFrame **);
	int GetFrame(IMultiSourceFrame *, IBodyFrame **);
	int GetFrame(IMultiSourceFrame *, IBodyIndexFrame **);
	int GetAudioSamples(IAudioBeamFrameReader *, float*, size_t);

	IKinectSensor* sensor;           // Kinect sensor
	IMultiSourceFrameReader* reader; // Kinect data source
	ICoordinateMapper* mapper;       // Converts between depth, color, and 3d coordinates

	IAudioBeamFrameReader * audioReader;

	CameraIntrinsics c_i;

	ColorSpacePoint * depth2rgb;    // Maps depth pixels to rgb pixels
	unsigned char * rgbimage;       // Stores RGB color image
	unsigned char * mappedRGBImage; // Stores RGB color image
	unsigned long sequence = 0;
    
	BODY_STRUCT body_buffers[BODY_COUNT];
private:
    
    std::string serial;
	std::thread * audio_thread;
	void GetAudio();
	bool _audio_running;
	static const size_t audio_buffer_size = 64000;
	float audio_buffer[audio_buffer_size];
	std::mutex audio_mutex;
	int buffer_loc = 0;

};

} } }
