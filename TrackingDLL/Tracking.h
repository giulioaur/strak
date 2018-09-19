#pragma once

#ifdef TRACKING_EXPORTS  
#define TRACKING_API __declspec(dllexport)   
#else  
#define TRACKING_API __declspec(dllimport)   
#endif  

#include <cstdint>

extern "C" {
	TRACKING_API int __stdcall init(const char *localAddress, int localPort, const char *remoteAddress, int remotePort, const char *cameraScriptsPath);
	TRACKING_API int __stdcall end();
	TRACKING_API int __stdcall getFeatures(uint16_t *buffer);
	TRACKING_API void __stdcall setColorFilter(int green, int blue);
	TRACKING_API void __stdcall setMiddleLine(int height);
	TRACKING_API void __stdcall startCalibration();
	TRACKING_API void __stdcall stopCalibration();
	TRACKING_API void __stdcall setSendFlag(bool flag);
	TRACKING_API void __stdcall getImages(uint8_t *img, uint8_t *normalized, uint8_t *lip);
	TRACKING_API int __stdcall getVideo(uint8_t *frame);


	//CAMERA_API int __stdcall decode(const AVPacket &packet, unsigned char *buffer);
}