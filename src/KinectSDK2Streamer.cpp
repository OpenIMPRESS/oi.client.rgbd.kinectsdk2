#include "KinectSDK2Streamer.hpp"

namespace oi { namespace core { namespace rgbd {

	KinectDSK2Streamer::KinectDSK2Streamer(StreamerConfig cfg, oi::core::network::UDPBase * c) : RGBDStreamer(cfg, c) {
	}

	bool KinectDSK2Streamer::OpenDevice() {
		HRESULT hres = S_OK;

		if (FAILED(GetDefaultKinectSensor(&sensor))) {
			return false;
		}

		if (sensor) {
			if (FAILED(sensor->Open())) {
				std::cout << "Failed to open " << std::endl; return -1;
			}

			if (FAILED(sensor->get_CoordinateMapper(&mapper))) {
				std::cout << "Failed to get coordinate mapper " << std::endl; return -1;
			}

			hres = sensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
				&reader);
			if (FAILED(hres)) {
				std::cout << "Failed to OpenMultiSourceFrameReader " << std::endl; return -1;
			}

			IAudioSource * audioSource;
			//IAudioBeamList * audioBeamList;

			hres = sensor->get_AudioSource(&audioSource);
			if (FAILED(hres)) {
				std::cout << "Failed to get_AudioSource " << std::endl; return -1;
			}

			hres = audioSource->OpenReader(&audioReader);
			if (FAILED(hres)) {
				std::cout << "Failed to OpenReader " << std::endl; return -1;
			}

			//hres = audioSource->get_AudioBeams(&audioBeamList);
			//hres = audioBeamList->OpenAudioBeam(0, &audioBeam);
			//hres = audioBeam->OpenInputStream(&audioStream);
			//if (audioBeamList) audioBeamList->Release();
			if (audioSource) audioSource->Release();
		} else {
			return false;
		}

		//wait a moment; GetDepthCameraIntrinsics can be unreliable at startup
		std::this_thread::sleep_for(std::chrono::microseconds(2500000));

		serial = this->config.deviceSerial;
		if (serial == "") {
			WCHAR uniqueKinectId[256];
			if (FAILED(sensor->get_UniqueKinectId(256, uniqueKinectId))) {
				std::cout << "NO ID" << std::endl;
			}

			char _serial[33];
			for (int i = 0; i < sizeof(_serial) / sizeof(stream_config.guid[0]); i++) {
				_serial[i] = uniqueKinectId[i];
			}
			
			serial = std::string(_serial);
		} else {
			std::cerr << "TODO: opening specific device by serial not implemented yet!" << std::endl;
		}

		if (FAILED(mapper->GetDepthCameraIntrinsics(&c_i))) {
			std::cout << "NO intrinsics" << std::endl;
		}

		depth2rgb = new ColorSpacePoint[frame_width()*frame_height()];
		rgbimage = new unsigned char[color_width() * color_height() * 4];
		mappedRGBImage = new unsigned char[frame_width()*frame_height() * 3];

		std::cout << "Device started." << std::endl;
		std::cout << "\tSerial:\t " << serial << std::endl;

		return true;
	}

	int KinectDSK2Streamer::SendFrame() {
		HRESULT hres = S_OK;

		size_t audio_buffer_loc = 0;
		float audio_buffer[1600];
		
		IAudioBeamFrameList * beamFrames;

		hres = audioReader->AcquireLatestBeamFrames(&beamFrames);
		if (beamFrames) {
			UINT beamCount = 0;
			hres = beamFrames->get_BeamCount(&beamCount);
			if (beamCount > 1) {
				std::cout << " >> " << beamCount << "\n";
			}
			for (int i = 0; i < beamCount; i++) {
				IAudioBeamFrame * beamFrame;
				hres = beamFrames->OpenAudioBeamFrame(i, &beamFrame);
				UINT subFrameCount = 0;
				hres = beamFrame->get_SubFrameCount(&subFrameCount);
				for (int j = 0; j < subFrameCount; j++) {
					IAudioBeamSubFrame * beamSubFrame;
					hres = beamFrame->GetSubFrame(j, &beamSubFrame);
					float beamAngle, beamConfidence;
					hres = beamSubFrame->get_BeamAngle(&beamAngle);
					hres = beamSubFrame->get_BeamAngleConfidence(&beamConfidence);
					//std::cout << beamAngle * 180 / 3.1415 << std::endl;
					float * audioBuffer = NULL;
					UINT cbRead = 0;
					hres = beamSubFrame->AccessUnderlyingBuffer(&cbRead, (BYTE **)&audioBuffer);
					memcpy(&(audio_buffer[audio_buffer_loc]), audioBuffer, cbRead);
					audio_buffer_loc += (size_t) (cbRead / sizeof(float));
					if (beamSubFrame != NULL) beamSubFrame->Release();
				}
				if (beamFrame != NULL) beamFrame->Release();
			}
		}
		if (beamFrames != NULL) beamFrames->Release();
		if (audio_buffer_loc > 0) {
			_SendAudioFrame(sequence, audio_buffer, audio_buffer_loc, 16000, 1);
		}
		/*
		DWORD cbRead = 0;
		hres = audioStream->Read((void *)audio_buffer, sizeof(audio_buffer), &cbRead);
		audio_buffer_loc = cbRead / sizeof(float);

		if (FAILED(hres) && hres != E_PENDING) {
			std::cerr << "Failed reading audio..." << std::endl;
			return 0;
		} else if (audio_buffer_loc > 0) {
			_SendAudioFrame(sequence, audio_buffer, audio_buffer_loc, 16000, 1);
		}*/


		IMultiSourceFrame* frame = NULL;
		hres = reader->AcquireLatestFrame(&frame);
		if (hres == E_PENDING) return 0; // PENDING
		if (!SUCCEEDED(hres)) {
			std::cout << "FAILED TO ACQUIRE FRAME: " << hres << std::endl;
			return -1;
		}

		IDepthFrame* depthFrame;
		IDepthFrameReference* depthframeref = NULL;
		frame->get_DepthFrameReference(&depthframeref);
		hres = depthframeref->AcquireFrame(&depthFrame);
		if (depthframeref) depthframeref->Release();
		if (hres == E_PENDING) return 0; // PENDING


		IColorFrame* colorFrame;
		IColorFrameReference* colorframeref = NULL;
		frame->get_ColorFrameReference(&colorframeref);
		hres = colorframeref->AcquireFrame(&colorFrame);
		if (colorframeref) colorframeref->Release();
		if (hres == E_PENDING) return 0; // PENDING

		if (!depthFrame || !colorFrame) {
			std::cout << "ERROR: depth/color frame undefined" << std::endl;
			return 0;
		}

		UINT depthCapacity = 0;
		UINT16 *depthData = NULL;
		depthFrame->AccessUnderlyingBuffer(&depthCapacity, &depthData);
		mapper->MapDepthFrameToColorSpace(
			depthCapacity, depthData,                  // Depth frame data and size of depth frame
			frame_width()*frame_height(), depth2rgb); // Output ColorSpacePoint array and size

		colorFrame->CopyConvertedFrameDataToArray(color_width() * color_height() * 4, rgbimage, ColorImageFormat_Rgba);

		for (int i = 0; i < frame_width()*frame_height(); i++) {
			ColorSpacePoint p = depth2rgb[i];
			if (p.X < 0 || p.Y < 0 || p.X > color_width() || p.Y > color_height()) {
				mappedRGBImage[3 * i + 0] = 128;
				mappedRGBImage[3 * i + 1] = 128;
				mappedRGBImage[3 * i + 2] = 128;
			} else {
				int idx = (int)p.X + color_width()*(int)p.Y;
				mappedRGBImage[3 * i + 0] = rgbimage[4 * idx + 0];
				mappedRGBImage[3 * i + 1] = rgbimage[4 * idx + 1];
				mappedRGBImage[3 * i + 2] = rgbimage[4 * idx + 2];
			}
		}


		int res = _SendFrame(++sequence, mappedRGBImage, (unsigned short*) depthData);

		depthFrame->Release();
		colorFrame->Release();

		return res;
	}

	bool KinectDSK2Streamer::CloseDevice() {
		return SUCCEEDED(sensor->Close());
	}

	bool KinectDSK2Streamer::HandleData(oi::core::network::DataContainer * dc) {
		return false;
	}

	int KinectDSK2Streamer::frame_width() {
		return 512;
	}

	int KinectDSK2Streamer::frame_height() {
		return 424;
	}
	
	int KinectDSK2Streamer::color_width() {
		return 1920;
	}

	int KinectDSK2Streamer::color_height() {
		return 1080;
	}

	int KinectDSK2Streamer::send_depth_stride() {
		return 2;
	}

	int KinectDSK2Streamer::raw_depth_stride() {
		return 2;
	}

	int KinectDSK2Streamer::raw_color_stride() {
		return 4;
	}

	float KinectDSK2Streamer::device_cx() {
		return c_i.PrincipalPointX;
	}

	float KinectDSK2Streamer::device_cy() {
		return c_i.PrincipalPointY;
	}

	float KinectDSK2Streamer::device_fx() {
		return c_i.FocalLengthX;
	}

	float KinectDSK2Streamer::device_fy() {
		return c_i.FocalLengthY;
	}

	float KinectDSK2Streamer::device_depth_scale() {
		return 0.001f;
	}

	std::string KinectDSK2Streamer::device_guid() {
		return serial;
	}

	TJPF KinectDSK2Streamer::color_pixel_format() {
		return TJPF_RGB;
	}

} } }