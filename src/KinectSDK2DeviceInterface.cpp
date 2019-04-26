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

#include "KinectSDK2DeviceInterface.hpp"
#include <comdef.h>

using namespace oi::client::kinectsdk2;
using namespace oi::core;


namespace oi {
	namespace client {
		namespace kinectsdk2 {
			template<class Interface>
			inline void SafeRelease(Interface *& IRelease) {
				if (IRelease != NULL) {
					IRelease->Release();
					IRelease = NULL;
				}
			}

			std::string format_error(unsigned __int32 hr) {
				std::stringstream ss;
				ss << "Failed to Initialize COM. Error code = 0x" << std::hex << hr << std::endl;
				return ss.str();
			}
		}
	}
}

KinectSDK2DeviceInterface::KinectSDK2DeviceInterface(std::string serial) {
    this->serial = serial;
	// Init class...
    OpenDevice();
}

oi::core::rgbd::RGBDDevice * mystreamer = nullptr;

int KinectSDK2DeviceInterface::Cycle(oi::core::rgbd::RGBDDevice * streamer) {
	int res = 0;
	mystreamer = streamer;

	std::chrono::milliseconds timestamp = NOW();

	HRESULT hres = S_OK;
	IMultiSourceFrame* frame = NULL;
	hres = reader->AcquireLatestFrame(&frame);
	if (hres == E_PENDING) return 0; // PENDING
	if (!SUCCEEDED(hres)) {
		_com_error err(hres);
		std::cerr << "\nERROR GETTING FRAME: " << err.ErrorMessage() << std::endl;
		std::cerr << format_error(hres) << std::endl;
		SafeRelease(frame);
		return -1;
	}

	IDepthFrame* depthFrame;
	int dRes = GetFrame(frame, &depthFrame);

	IColorFrame* colorFrame;
	int cRes = GetFrame(frame, &colorFrame);

	IBodyFrame * bodyFrame;
	int bRes = GetFrame(frame, &bodyFrame);

	IBodyIndexFrame * bodyIndexFrame;
	int biRes = GetFrame(frame, &bodyIndexFrame);

	if (dRes + cRes + bRes + biRes != 4) {
		SafeRelease(bodyIndexFrame);
		SafeRelease(bodyFrame);
		SafeRelease(colorFrame);
		SafeRelease(depthFrame);

		if (dRes == 0 && cRes == 0 && dRes == 0 && biRes == 0) {
			std::cout << "ALL PENDING" << std::endl;
			// All pending
			return 0;
		}
		std::cout << "SOME PENDING: depth: " << dRes << " color: " << cRes << " body: " << bRes << " bodyIndex: " << biRes << std::endl;

		return 0; // If some are pending, some are not, something is wrong.
	}

	// PREPARING RGBD DATA
	UINT depthCapacity = 0;
	UINT16 *depthData = NULL;
	hres = depthFrame->AccessUnderlyingBuffer(&depthCapacity, &depthData);

	UINT bodyIndexBufferSize = 0;
	BYTE * bodyIndexBuffer = NULL;
	hres = bodyIndexFrame->AccessUnderlyingBuffer(&bodyIndexBufferSize, &bodyIndexBuffer);


	unsigned char * depthMult = new unsigned char[bodyIndexBufferSize];
	int bidx = 0;
	while (bidx < bodyIndexBufferSize) {
		if (bodyIndexBuffer[bidx] > 5) depthMult[bidx] = 0;
		else                     depthMult[bidx] = 255 - bodyIndexBuffer[bidx] * 40;
		bidx++;
	}

	streamer->QueueBodyIndexFrame(depthMult, frame_width(), frame_height(), TJPF_GRAY, timestamp);
	delete[] depthMult;


	if (SUCCEEDED(hres)) {
		//mapper->MapColorFrameToDepthSpace();
		hres = mapper->MapDepthFrameToColorSpace(
			depthCapacity, depthData,                  // Depth frame data and size of depth frame
			frame_width()*frame_height(), depth2rgb); // Output ColorSpacePoint array and size
		if (SUCCEEDED(hres)) {
			hres = colorFrame->CopyConvertedFrameDataToArray(color_width() * color_height() * 4, rgbimage, ColorImageFormat_Rgba);
			if (SUCCEEDED(hres)) {

				// PREPARING BODY DATA
				//_SendHDFrame(rgbimage, color_width(), color_height(), TJPF_RGBA, timestamp);


				IBody* bodies[BODY_COUNT] = { 0 };
				INT64 nTime = 0;
				hres = bodyFrame->get_RelativeTime(&nTime);
				hres = bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);
				if (SUCCEEDED(hres)) {
					for (int i = 0; i < frame_width()*frame_height(); i++) {
						ColorSpacePoint p = depth2rgb[i];
						if (p.X < 0 || p.Y < 0 || p.X > color_width() || p.Y > color_height()) { // || bodyIndexBuffer[i] > 5
							mappedRGBImage[3 * i + 0] = 0; // 
							mappedRGBImage[3 * i + 1] = 0; // 
							mappedRGBImage[3 * i + 2] = 0; //
						}
						else {
							int idx = (int)p.X + color_width()*(int)p.Y;
							mappedRGBImage[3 * i + 0] = rgbimage[4 * idx + 0];
							mappedRGBImage[3 * i + 1] = rgbimage[4 * idx + 1];
							mappedRGBImage[3 * i + 2] = rgbimage[4 * idx + 2];
						}
					}
					// TODO: kinect sdk sequence id?
					res += streamer->QueueRGBDFrame(++sequence, mappedRGBImage, (unsigned short*)depthData, timestamp);

					unsigned short send_bodies = 0;
					for (int i = 0; i < _countof(bodies); ++i) {
						IBody* body = bodies[i];
						if (!body) continue;
						BOOLEAN is_tracked = false;
						hres = body->get_IsTracked(&is_tracked);
						if (!SUCCEEDED(hres)) {
							std::cerr << "\nERROR: FAILED TO get_IsTracked" << std::endl;
							continue;
						}
						if (!is_tracked) continue;
						Joint joints[JointType_Count];
						hres = body->GetJoints(_countof(joints), joints);
						if (!SUCCEEDED(hres)) {
							std::cerr << "\nERROR: FAILED TO GetJoints" << std::endl;
							continue;
						}

						HandState leftHandState = HandState_Unknown;
						HandState rightHandState = HandState_Unknown;
						DetectionResult drengaged = DetectionResult_Unknown;
						PointF lean;
						TrackingState tslean;
						UINT64 trackingID;

						body->get_HandLeftState(&leftHandState);
						body->get_HandRightState(&rightHandState);
						body->get_Lean(&lean);
						body->get_LeanTrackingState(&tslean);
						body->get_TrackingId(&trackingID);

						body_buffers[send_bodies].left_hand_state = (unsigned char)leftHandState;
						body_buffers[send_bodies].right_hand_state = (unsigned char)rightHandState;
						body_buffers[send_bodies].leanX = lean.X;
						body_buffers[send_bodies].leanY = lean.Y;
						body_buffers[send_bodies].lean_tracking_state = (unsigned char)tslean;
						body_buffers[send_bodies].tracking_id = (unsigned long)trackingID;

						// joint order is know, we will just send a
						for (int j = 0; j < _countof(joints); ++j) {
							body_buffers[send_bodies].joints_position[j * 3 + 0] = joints[j].Position.X;
							body_buffers[send_bodies].joints_position[j * 3 + 1] = joints[j].Position.Y;
							body_buffers[send_bodies].joints_position[j * 3 + 2] = joints[j].Position.Z;
							body_buffers[send_bodies].joints_tracked[j] = (unsigned char)joints[j].TrackingState;
						}
						send_bodies++;
					}

					// SENDING BODY DATA
					res += streamer->QueueBodyFrame(&(body_buffers[0]), send_bodies, timestamp); //&body_buffers, send_bodies

					for (int i = 0; i < _countof(bodies); ++i) {
						SafeRelease(bodies[i]);
					}
				} else std::cerr << "\nERROR: GetAndRefreshBodyData" << std::endl;
			} else std::cerr << "\nERROR: CopyConvertedFrameDataToArray" << std::endl;
		} else std::cerr << "\nERROR: MapDepthFrameToColorSpace" << std::endl;
	} else std::cerr << "\nERROR: AccessUnderlyingBuffer" << std::endl;

	SafeRelease(colorFrame);
	SafeRelease(depthFrame);
	SafeRelease(bodyFrame);
	SafeRelease(bodyIndexFrame);
	SafeRelease(frame);

    return res;
}


int KinectSDK2DeviceInterface::GetFrame(IMultiSourceFrame * frame, IColorFrame ** res) {
	IColorFrameReference* colorframeref = NULL;
	frame->get_ColorFrameReference(&colorframeref);
	HRESULT hres = colorframeref->AcquireFrame(res);
	SafeRelease(colorframeref);
	if (hres == E_PENDING) return 0;
	else if (!SUCCEEDED(hres)) {
		if (true) { // debug level
			_com_error err(hres);
			std::cerr << "\nERROR GETTING COLOR FRAME: " << err.ErrorMessage() << std::endl;
			std::cerr << format_error(hres) << std::endl;
		}
		return -1;
	}
	return 1;
}

int KinectSDK2DeviceInterface::GetFrame(IMultiSourceFrame * frame, IDepthFrame ** res) {
	IDepthFrameReference* depthframeref = NULL;
	frame->get_DepthFrameReference(&depthframeref);
	HRESULT hres = depthframeref->AcquireFrame(res);
	SafeRelease(depthframeref);
	if (hres == E_PENDING) return 0;
	else if (!SUCCEEDED(hres)) {
		if (true) { // debug level
			_com_error err(hres);
			std::cerr << "\nERROR GETTING DEPTH FRAME: " << err.ErrorMessage() << std::endl;
			std::cerr << format_error(hres) << std::endl;
		}
		return -1;
	}
	else return 1;
}

int KinectSDK2DeviceInterface::GetFrame(IMultiSourceFrame * frame, IBodyFrame ** res) {
	IBodyFrameReference * bodyFrameRef = NULL;
	frame->get_BodyFrameReference(&bodyFrameRef);
	HRESULT hres = bodyFrameRef->AcquireFrame(res);
	SafeRelease(bodyFrameRef);
	if (hres == E_PENDING) return 0;
	else if (!SUCCEEDED(hres)) {
		if (true) { // debug level
			_com_error err(hres);
			std::cerr << "\nERROR GETTING BODY FRAME: " << err.ErrorMessage() << std::endl;
			std::cerr << format_error(hres) << std::endl;
		}
		return -1;
	}
	return 1;
}

int KinectSDK2DeviceInterface::GetFrame(IMultiSourceFrame * frame, IBodyIndexFrame ** res) {
	IBodyIndexFrameReference * bodyIndexFrameRef = NULL;
	frame->get_BodyIndexFrameReference(&bodyIndexFrameRef);
	HRESULT hres = bodyIndexFrameRef->AcquireFrame(res);
	SafeRelease(bodyIndexFrameRef);
	if (hres == E_PENDING) return 0;
	else if (!SUCCEEDED(hres)) {
		if (true) { // debug level
			_com_error err(hres);
			std::cerr << "\nERROR GETTING BODY INDEX FRAME: " << err.ErrorMessage() << std::endl;
			std::cerr << format_error(hres) << std::endl;
		}
		return -1;
	}
	return 1;
}

int KinectSDK2DeviceInterface::OpenDevice() {
	HRESULT hres = S_OK;
	std::cout << "Opening devive..." << std::endl;
	hres = GetDefaultKinectSensor(&sensor);
	if (FAILED(hres)) {
		return false;
	}

	if (sensor) {
		if (FAILED(sensor->Open())) {
			std::cout << "Failed to open " << std::endl; return false;
		}

		if (FAILED(sensor->get_CoordinateMapper(&mapper))) {
			std::cout << "Failed to get coordinate mapper " << std::endl; return false;
		}

		hres = sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Body | FrameSourceTypes::FrameSourceTypes_BodyIndex,
			&reader);
		if (FAILED(hres)) {
			std::cout << "Failed to OpenMultiSourceFrameReader " << std::endl; return false;
		}

		IAudioSource *			pAudioSource;
		IAudioBeamList* pAudioBeamList;
		if (SUCCEEDED(hres)) {
			hres = sensor->get_AudioSource(&pAudioSource);
		}

		if (SUCCEEDED(hres)) {
			hres = pAudioSource->get_AudioBeams(&pAudioBeamList);
		}

		if (SUCCEEDED(hres)) {
			hres = pAudioBeamList->OpenAudioBeam(0, &m_pAudioBeam);
		}

		if (SUCCEEDED(hres)) {
			/*
			hres = m_pAudioBeam->put_AudioBeamMode(AudioBeamMode_Manual);
			hres = m_pAudioBeam->put_BeamAngle(180);
			*/
			hres = m_pAudioBeam->OpenInputStream(&m_pAudioStream);
		}

		SafeRelease(pAudioBeamList);
		SafeRelease(pAudioSource);

	}
	else {
		return false;
	}

	//wait a moment; GetDepthCameraIntrinsics can be unreliable at startup
	std::this_thread::sleep_for(std::chrono::microseconds(2500000));


	if (serial == "") {
		std::vector<WCHAR> kinect_uid_wbuffer(256, 0);
		if (FAILED(sensor->get_UniqueKinectId(kinect_uid_wbuffer.size(), kinect_uid_wbuffer.data()))) {
			std::cout << "NO ID" << std::endl;
			return false;
		}

		for (size_t i = 0; i < kinect_uid_wbuffer.size(); ++i) {
			std::cout << kinect_uid_wbuffer.at(i);
		}
		std::cout << std::endl;

		while (!kinect_uid_wbuffer.empty() && kinect_uid_wbuffer.back() == 0) {
			kinect_uid_wbuffer.pop_back();
		}

		std::string kinect_uid(kinect_uid_wbuffer.size(), '0');
		for (size_t i = 0; i < kinect_uid_wbuffer.size(); ++i) {
			kinect_uid[i] = static_cast<char>(kinect_uid_wbuffer.at(i));
		}

		if (kinect_uid.empty()) {
			std::cout << "ERROR: Couldn't get device serial." << std::endl;
			return false;
		}
		serial = kinect_uid;
	} else {
		std::cerr << "TODO: opening specific device by serial not implemented yet!" << std::endl;
	}

	if (FAILED(mapper->GetDepthCameraIntrinsics(&c_i))) {
		std::cout << "NO intrinsics" << std::endl;
	}

	depth2rgb = new ColorSpacePoint[frame_width() * frame_height()];
	rgbimage = new unsigned char[color_width() * color_height() * 4];
	mappedRGBImage = new unsigned char[frame_width()*frame_height() * 3];

	std::cout << "Device started." << std::endl;
	std::cout << "\tSerial:\t " << serial << std::endl;

	audio_thread = new std::thread(&KinectSDK2DeviceInterface::GetAudio, this);

	return true;
}


void KinectSDK2DeviceInterface::GetAudio() {
	HRESULT hres = S_OK;
	_audio_running = true;
	while (_audio_running) {
		static float audioBuffer[cAudioBufferLength];
		DWORD cbRead = 0;
		hres = m_pAudioStream->Read((void *)audioBuffer, sizeof(audioBuffer), &cbRead);
		if (FAILED(hres) && hres != E_PENDING) {
		} else if (cbRead > 0 && mystreamer != nullptr) {
			std::chrono::milliseconds timestamp = NOW();
			mystreamer->QueueAudioFrame(sequence, &(audioBuffer[0]), (size_t)(cbRead / sizeof(float)), 16000, 1, timestamp);
		}
		std::this_thread::sleep_for(std::chrono::microseconds(cAudioReadTimerInterval));
	}
}

int KinectSDK2DeviceInterface::CloseDevice() {
	if (sensor != NULL && SUCCEEDED(sensor->Close())) {
		std::cout << "Closed Kinect successfully" << std::endl;
		return 1;
	} else {
		std::cout << "Failed closing Kinect" << std::endl;
		return -1;
	}
}

int KinectSDK2DeviceInterface::color_width() {
	return 1920;
}

int KinectSDK2DeviceInterface::color_height() {
	return 1080;
}

int KinectSDK2DeviceInterface::frame_width() {
    return 512;
}

int KinectSDK2DeviceInterface::frame_height() {
    return 424;
}

int KinectSDK2DeviceInterface::send_depth_stride() {
    return 2;
}

int KinectSDK2DeviceInterface::raw_depth_stride() {
    return 2;
}

int KinectSDK2DeviceInterface::raw_color_stride() {
    return 4;
}

float KinectSDK2DeviceInterface::device_cx() {
	return c_i.PrincipalPointX;
}

float KinectSDK2DeviceInterface::device_cy() {
	return c_i.PrincipalPointY;
}

float KinectSDK2DeviceInterface::device_fx() {
	return c_i.FocalLengthX;
}

float KinectSDK2DeviceInterface::device_fy() {
	return c_i.FocalLengthY;
}

float KinectSDK2DeviceInterface::device_depth_scale() {
	return 0.001f;
}

std::string KinectSDK2DeviceInterface::device_guid() {
    return serial;
}

TJPF KinectSDK2DeviceInterface::color_pixel_format() {
	return TJPF_RGB;
}
bool KinectSDK2DeviceInterface::supports_depth() {
	return true;
}

bool KinectSDK2DeviceInterface::supports_audio() {
	return true;
}

bool KinectSDK2DeviceInterface::supports_body() {
	return true;
}

bool KinectSDK2DeviceInterface::supports_bidx() {
	return true;
}

bool KinectSDK2DeviceInterface::supports_hd() {
	return false;
	//return true;
}
