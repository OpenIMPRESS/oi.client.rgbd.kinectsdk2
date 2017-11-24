#pragma once


#include <stdlib.h>
#include <stdio.h>
#include <string>
extern "C" {
#include <libavcodec\avcodec.h>
#include <libavformat\avformat.h>
#include <libavutil\mathematics.h>
}
namespace oi { namespace util { namespace av {

	class AVEncodeTest {
	public:
		AVEncodeTest(std::string filename);
		int AddFrame(unsigned char * rgbdata);
		int Close();
		AVCodec *codec;
		AVCodecContext *c = NULL;
		int i, ret, x, y, got_output;
		uint8_t *frame_buf;
		FILE *f;
		AVFrame *frame;
		AVPacket pkt;
		AVCodecID codec_id = AV_CODEC_ID_MPEG1VIDEO;
		bool works = false;
		int fcount;
	};

} } }