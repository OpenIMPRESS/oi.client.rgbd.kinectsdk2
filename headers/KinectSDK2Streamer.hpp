#pragma once
#include <asio.hpp>
#include <turbojpeg.h>
#include <cstdlib>
#include <signal.h>
#include <Kinect.h>

#include <UDPConnector.hpp>
#include <RGBDStreamer.hpp>

namespace oi { namespace core { namespace rgbd {

	template<class Interface>
	inline void SafeRelease(Interface *& IRelease);

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
		bool supports_body();
		bool supports_bidx();
		bool supports_hd();

		// 0 for pending, 1 for good, -X for error
		int GetFrame(IMultiSourceFrame *, IColorFrame **);
		int GetFrame(IMultiSourceFrame *, IDepthFrame **);
		int GetFrame(IMultiSourceFrame *, IBodyFrame **);
		int GetFrame(IMultiSourceFrame *, IBodyIndexFrame **);
		int GetAudioSamples(IAudioBeamFrameReader *, float*, size_t);

		IKinectSensor* sensor;         // Kinect sensor
		IMultiSourceFrameReader* reader;   // Kinect data source
		ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates

		IAudioBeamFrameReader * audioReader;

		CameraIntrinsics c_i;

		ColorSpacePoint * depth2rgb;     // Maps depth pixels to rgb pixels
		unsigned char * rgbimage;    // Stores RGB color image
		unsigned char * mappedRGBImage;    // Stores RGB color image
		unsigned long sequence = 0;


		BODY_STRUCT body_buffers[BODY_COUNT];
	private:
		//TIMESPAN _expected_next_audio_frame_start = 0;

		std::thread * audio_thread;
		void GetAudio();
		bool _audio_running;
		static const size_t audio_buffer_size = 64000;
		float audio_buffer[audio_buffer_size];
		std::mutex audio_mutex;
		int buffer_loc = 0;


	};

} } }