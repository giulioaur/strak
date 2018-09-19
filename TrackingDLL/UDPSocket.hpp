#pragma once

#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <string>
#include <iostream>

/* networking libraries */
#include <WinSock2.h>
#include <Ws2tcpip.h>
#include <process.h> 
#include "Dwmapi.h"
#include <Iphlpapi.h>
#include <Wlanapi.h>

#include<windows.h>

namespace strak {
class UDPSocket {
	private:

		SOCKET sock;

	public:

		// Constructors.
		UDPSocket();
		~UDPSocket();

		// Gets the win socket.
		int getSocket();

		// Binds the socket.
		bool bind(const std::string &address, const int port);

		// Send data to an address.
		int sendTo(const std::string& address, unsigned short port, const char* buffer, int len, int flags = 0);

		// Receive data from an address.
		bool receiveFrom(char* buffer, int &len, int flags = 0);
};
}

