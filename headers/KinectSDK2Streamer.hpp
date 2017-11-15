#pragma once
#include <asio.hpp>
#include <turbojpeg.h>
#include <cstdlib>
#include <signal.h>
#include <Kinect.h>

#include <UDPConnector.hpp>
#include <RGBDStreamer.hpp>

namespace oi { namespace core { namespace rgbd {

	class KinectDSK2Streamer : public RGBDStreamer {
	public:
		KinectDSK2Streamer(StreamerConfig cfg, oi::core::network::UDPBase * c);
		bool OpenDevice();
		bool CloseDevice();
		bool HandleData(oi::core::network::DataContainer * dc) ;
		int SendFrame();
	protected:
		int frame_width();
		int frame_height();
		int color_width();
		int color_height();
		int send_depth_stride();
		int raw_depth_stride();
		int raw_color_stride();

		float device_cx();
		float device_cy();
		float device_fx();
		float device_fy();
		float device_depth_scale();
		std::string serial;
		std::string device_guid();
		TJPF color_pixel_format();
		bool supports_audio();

		IKinectSensor* sensor;         // Kinect sensor
		IMultiSourceFrameReader* reader;   // Kinect data source
		ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates



		IAudioBeamFrameReader * audioReader;
		//IAudioBeam *     audioBeam;
		//IStream * audioStream;

		CameraIntrinsics c_i;

		ColorSpacePoint * depth2rgb;     // Maps depth pixels to rgb pixels
		unsigned char * rgbimage;    // Stores RGB color image
		unsigned char * mappedRGBImage;    // Stores RGB color image
		unsigned long sequence = 0;
	};

} } }