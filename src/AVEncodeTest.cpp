#include "AVEncodeTest.hpp"

#ifdef HAVE_AV_CONFIG_H
#undef HAVE_AV_CONFIG_H
#endif
#define INBUF_SIZE 4096
#define AUDIO_INBUF_SIZE 20480
#define AUDIO_REFILL_THRESH 4096

namespace oi { namespace util { namespace av {

	AVEncodeTest::AVEncodeTest(std::string filename) {
		/*
		avcodec_register_all();
		codec = avcodec_find_encoder(codec_id);
		if (!codec) {
			fprintf(stderr, "codec not found\n");
			return;
		}
		
		c = avcodec_alloc_context3(codec);
		if (!c) {
			fprintf(stderr, "Could not allocate video codec context\n");
			return;
		}

		// put sample parameters 
		c->bit_rate = 800000;
		// resolution must be a multiple of two 
		c->width = 1920;
		c->height = 1080;
		c->time_base.num = 1;
		c->time_base.den = 25;
		c->gop_size = 10; // emit one intra frame every ten frames 
		c->max_b_frames = 1;
		c->pix_fmt = AV_PIX_FMT_YUV420P;

		AVDictionary *codec_options = NULL;

		if (codec->id == AV_CODEC_ID_H264) {
			av_dict_set(&codec_options, "preset", "veryfast", 0);
		}

		// open it
		if (avcodec_open2(c, codec, &codec_options) < 0) {
			fprintf(stderr, "could not open codec\n");
			return;
		}

		f = fopen(filename.c_str(), "wb");
		if (!f) {
			fprintf(stderr, "could not open %s\n", filename);
			return;
		}
		

		frame = av_frame_alloc();
		if (!frame) {
			fprintf(stderr, "Could not allocate video frame\n");
			return;
		}
		frame->format = AV_PIX_FMT_RGBA;
		frame->width = c->width;
		frame->height = c->height;
		frame->nb_samples = 0;
		if (av_frame_get_buffer(frame, 1) < 0) {
			fprintf(stderr, "Could not allocate video frame buffer\n");
			return;
		}
		fcount = 0;
		works = true;
		fprintf(stdout, "Encoding initialized!\n");
		*/
	}

	int AVEncodeTest::AddFrame(unsigned char * rgbdata) {
		/*
		if (!works) {
			fprintf(stderr, "ERROR: Can't add frame!\n");
			return -1;
		}
		av_init_packet(&pkt);
		pkt.data = NULL;
		pkt.size = 0;

		memset(frame->data[0], 0, frame->width * frame->height * 4);
		//uint8_t *pix_ptr = frame->data[0];
		//memcpy(&(frame->data[0][0]), rgbdata, frame->width * frame->height * 4);
		frame->pts = fcount;

		ret = avcodec_encode_video2(c, &pkt, frame, &got_output);
		if (ret < 0) {
			fprintf(stderr, "Error encoding frame\n");
			return -1;
		}
		
		if (got_output) {
			printf("Write frame %3d (size=%5d)\n", i, pkt.size);
			fwrite(pkt.data, 1, pkt.size, f);
			av_free_packet(&pkt);
		}

		fcount++;
		return ret;
		*/
		return 0;
	}

	int AVEncodeTest::Close() {
		/*
		// get the delayed frames 
		if (!works) {
			fprintf(stderr, "Error: Didn't save file\n");
			return 0;
		}
		fprintf(stdout, "Tryign to save...\n");
		for (got_output = 1; got_output; i++) {
			ret = avcodec_encode_video2(c, &pkt, NULL, &got_output);
			if (ret < 0) {
				fprintf(stderr, "Error encoding frame\n");
				return -1;
			}

			if (got_output) {
				printf("Write frame %3d (size=%5d)\n", i, pkt.size);
				fwrite(pkt.data, 1, pkt.size, f);
				av_free_packet(&pkt);
			}
		}

		// add sequence end code to have a real mpeg file 
		uint8_t endcode[] = { 0, 0, 1, 0xb7 };
		fwrite(endcode, 1, sizeof(endcode), f);
		fclose(f);
		avcodec_close(c);
		av_free(c);
		av_freep(&frame->data[0]);
		av_frame_free(&frame);
		printf("\n");
		*/
		return 0;
	}

} } }