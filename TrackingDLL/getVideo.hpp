#pragma once

extern "C" {
#include "libavcodec/avcodec.h"
}

namespace strak {
/********************************************* GLOBAL CONSTANT *************************************************/

//const uint16_t imageWidth = 640, imageHeight = 480;
const uint32_t imageWidth = 800, imageHeight = 600;
const uint32_t BUFFER_SIZE = imageWidth * imageHeight * 3;

/********************************************* FUNCTIONS HEADERS *************************************************/

int init_network();
int end_network();
int init_decoder();
int end_decoder();
AVPacket receive(const int receiverSocket);
int decode(const AVPacket &packet, unsigned char *buffer);

}