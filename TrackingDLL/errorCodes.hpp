#pragma once
//#define CAMERA_OK 0
//#define CAMERA_NO_NETWORK -1
//#define CAMERA_NO_CODEC -2
//#define CAMERA_NO_CODEC_CONTEXT -3
//#define CAMERA_SOCKET_BIND -4
//#define CAMERA_NO_RECEIVE -5
//#define CAMERA_NO_SEND -6
//#define CAMERA_NO_RASP -7
//#define CAMERA_NO_FRAME -8
//#define CAMERA_UNKNOWN -9

enum TrackingError {
	OK = 0, 
	NO_NETWORK = - 1,
	NO_CODEC = - 2, 
	NO_CODEC_CONTEXT = - 3,
	SOCKET_BIND = - 4,
	NO_RECEIVE = - 5,
	NO_SEND = - 6,
	NO_RASP = - 7,
	NO_FRAME = - 8,
	UNKNOWN = - 9
};