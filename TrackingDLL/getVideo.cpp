#include "getVideo.hpp"
#include <array>
#include "UDPSocket.hpp"
#include "errorCodes.hpp"

/* decoder libraries */
extern "C" {
#include "libavcodec/avcodec.h"
#include "libavutil/opt.h"
#include "libswscale/swscale.h"
#include "libavformat/avformat.h"
}

/************************************ DEFINES ************************************/
#define SLOT_SIZE 400
#define SLOTS 512 
#define SLOT_DUPLICATE 1
#define SLOT_DUPLICATE_END 2

#define CODEC AV_CODEC_ID_H264
#define H264_INBUF_SIZE 16384  

using namespace std;

namespace strak {
/*****UDP STUFF*****************************************************************************************/
int current_size;
char tempbuffer[SLOT_SIZE + 6];
char* final_buffer;
bool slot_map[SLOTS];
int last_fragment;
int atom_size;
int last_counter;
int previous_fragment;
int engaged;
int verbose = 0;

/******************************************************************************************************************************/
AVCodec* decoder;
AVCodecContext* c_decoder;
AVFrame* frame_decoder;
AVFrame* frame_decoder_RGB;
SwsContext* img_convert_ctx;
unsigned char* inbuf;
unsigned char* temp_inbuf;
int temp_size;
bool first_video_pkt;

int init_network() {
	/* initialize networking */
	WSADATA wsadata;
	int init = WSAStartup(0x0202, &wsadata);
	if (init != 0)
		return TrackingError::NO_NETWORK;

	current_size = 0;
	last_fragment = -1;
	atom_size = 0;
	last_counter = -1;
	previous_fragment = -1;
	engaged = 0;
	first_video_pkt = true;
	inbuf = new unsigned char[500000];
	temp_inbuf = new unsigned char[500000];

	return TrackingError::OK;
}


int end_network() {
	WSACleanup();

	delete[]inbuf;
	delete[]temp_inbuf;
	return TrackingError::OK;
}

int init_decoder() {
	/* initialize decoder */
	avcodec_register_all();
	uint8_t *frame_rgb;
	int numBytes;
	AVPixelFormat pFormat = AV_PIX_FMT_BGR24;
	numBytes = avpicture_get_size(pFormat, imageWidth, imageHeight); // AV_PIX_FMT_RGBA
	frame_rgb = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));

	decoder = avcodec_find_decoder(CODEC);
	if (!decoder)
		return TrackingError::NO_CODEC;

	c_decoder = avcodec_alloc_context3(decoder);
	int val = avcodec_open2(c_decoder, decoder, NULL);
	if (val < 0)
		return TrackingError::NO_CODEC_CONTEXT;

	frame_decoder = av_frame_alloc();
	frame_decoder_RGB = av_frame_alloc();
	avpicture_fill((AVPicture*)frame_decoder_RGB, frame_rgb, pFormat, imageWidth, imageHeight);

	img_convert_ctx = sws_getContext(imageWidth, imageHeight, AV_PIX_FMT_YUV420P, imageWidth, imageHeight, pFormat, SWS_FAST_BILINEAR, NULL, NULL, NULL);

	return TrackingError::OK;
}

int end_decoder() {
	return TrackingError::OK;
}

/// Receives the packets from the raspberry.
int myrecv(const int receiverSocket, char *buffer, int sizemax, struct sockaddr_in *source = NULL, int *source_len = NULL) {
	int n;
	unsigned short int slot;
	unsigned char flags;
	unsigned short int counter;
	int ntotal = 0;
	//u_long mode = 0; // non-blocking mode (default)

	/* reset buffer in case of size change */
	if (sizemax > current_size) {
		if (final_buffer)
			delete final_buffer;
		final_buffer = new char[SLOT_SIZE * SLOTS];
		current_size = sizemax;
		for (int i = 0; i < SLOTS; i++)
			slot_map[i] = false;
	}

	/* switch to non-blocking mode */
	//ioctlsocket(receiveSocket, FIONBIO, &mode);

	/* receive all buffered data */
	do {
		/* receive data from socket */
		n = recv(receiverSocket, tempbuffer, SLOT_SIZE + 4, 0);

		if (n > 0) {
			/* check for invalid packet size */
			if (n < 4 || n > SLOT_SIZE + 4)
				continue;

			/* store header data */
			counter = ((unsigned char*)tempbuffer)[0];
			slot = ((unsigned short int *)tempbuffer)[1];
			flags = ((unsigned char*)tempbuffer)[1];

			/* check for control messages or invalid duplicates */
			if (counter == 0)
				continue;
			if ((flags & SLOT_DUPLICATE) && !engaged)
				continue;

			/* check for missing data in the current packet */
			if (engaged) {
				if (counter != last_counter) { // data missing from engaged packet
					for (int i = 0; i < previous_fragment; i++)
						if (verbose)
							if (slot_map[i])
								cout << "+";
					if (verbose)
						cout << "? HOLE: " << counter - 1 << endl;
					for (int i = 0; i < SLOTS; i++)
						slot_map[i] = false; // reset all
					last_fragment = -1;
					ntotal = -1; // report error to caller
					engaged = 0;
				}
			}
			else
				engaged = 1;

			/* store data in the local buffer */
			if (flags & SLOT_DUPLICATE && n == SLOT_SIZE + 4) {
				if (verbose)
					if (!slot_map[slot])
						cout << "C";
				memcpy(final_buffer + SLOT_SIZE * slot, tempbuffer + 4, n - 4);
				slot_map[slot] = true;
				last_counter = counter;
			}
			else {
				if (n < SLOT_SIZE + 4) {
					last_fragment = slot;
					atom_size = SLOT_SIZE * slot + n - 4;
				}
				memcpy(final_buffer + SLOT_SIZE * slot, tempbuffer + 4, n - 4);
				slot_map[slot] = true;
				last_counter = counter;
				previous_fragment = slot;
			}

			/* check if a packet is ready */
			if (last_fragment >= 0) { // packet tail
				int count = 0;
				for (int i = 0; i <= last_fragment; i++) {
					if (slot_map[i]) {
						count++;
						if (verbose)
							cout << "+";
					}
					else if (verbose)
						cout << "-";
				}
				if (count == last_fragment + 1) { // all data valid
					for (int i = 0; i < SLOTS; i++)
						slot_map[i] = false; // reset all
					last_fragment = -1;
					ntotal = atom_size;
					atom_size = 0;
					engaged = 0;
					memcpy(buffer, final_buffer, ntotal);
					if (verbose)
						cout << "OK! " << last_counter << endl;
				}
				else {
					if (verbose)
						cout << "? HOLE: " << last_counter << endl;
					engaged = 0;
					for (int i = 0; i < SLOTS; i++)
						slot_map[i] = false; // reset all
					last_fragment = -1;
					ntotal = -1; // notify hole
				}
			}
		}
	} while (n > 0 && ntotal == 0);

	if (n < 0) {
		cout << WSAGetLastError() << endl;
		system("pause");
	}

	return ntotal;
}

AVPacket receive(const int receiverSocket) {
	int bytes_read = -1;
	bool error = true;
	AVPacket packets;

	while (error) {
		error = false;

		// Read data until something is received.
		do {
			bytes_read = myrecv(receiverSocket, (char *)inbuf, 300000, NULL, NULL);
		} while (bytes_read == 0 || bytes_read == -1);

		if (inbuf[0] == 'n' || first_video_pkt == true) {
			bytes_read--; // levo il primo byte che è 'n'
			memcpy(temp_inbuf + temp_size, inbuf + 1, (bytes_read));
			temp_size += bytes_read;
			first_video_pkt = false;
			error = true;
		}
	}

	// Read received packets.
	av_init_packet(&packets);
	if (temp_size > 0) {
		bytes_read = bytes_read - 1; //levo  1 byte di 'y'
		memcpy(temp_inbuf + temp_size, inbuf + 1, bytes_read);
		packets.data = temp_inbuf; //data
		packets.size = bytes_read + temp_size; //size
		temp_size = 0;
	}
	else {
		packets.data = (inbuf + 1); //data
		packets.size = (bytes_read - 1); //size
	}

	return packets;
}

/// Decodes the received frames.
int decode(const AVPacket &packet, unsigned char *buffer) {
	int got_picture = 0;
	int len_2 = 0;

	// decode frame
	len_2 = avcodec_decode_video2(c_decoder, frame_decoder, &got_picture, &packet); //got_picture == 0 -> no frame decompressed
	if (len_2 <= 0) {
		return TrackingError::NO_FRAME;
	}

	if (got_picture != 0) {
		// scale image
		int ret = sws_scale(img_convert_ctx, frame_decoder->data, frame_decoder->linesize, 0, imageHeight,
			frame_decoder_RGB->data, frame_decoder_RGB->linesize);

		// copy image to shared buffer and update frame count.

		memcpy(buffer, frame_decoder_RGB->data[0], BUFFER_SIZE);

		return TrackingError::OK;
	}

	return TrackingError::UNKNOWN;
}
}