#include "UDPSocket.hpp"

#pragma comment(lib, "Iphlpapi.lib")
#pragma comment(lib, "Wlanapi.lib")
#pragma comment(lib,"comctl32.lib")
#pragma comment(lib,"winmm.lib")
#pragma comment(lib, "ws2_32.lib") 
#pragma comment(lib, "Dwmapi.lib") 

#include "UDPSocket.hpp"
namespace strak {
	// Constructors.
	UDPSocket::UDPSocket() {
		// Open socket.
		this->sock = socket(PF_INET, SOCK_DGRAM, 0);

		if (sock == INVALID_SOCKET)
			throw std::system_error(WSAGetLastError(), std::system_category(), "Could not create a new socket");
	}

	// Destructors.
	UDPSocket::~UDPSocket() {
		closesocket(sock);
	}

	// Gets the win socket.
	int UDPSocket::getSocket() {
		return this->sock;
	}

	// Binds the socket.
	bool UDPSocket::bind(const std::string &address, const int port) {
		struct sockaddr_in si_other;
		memset((char *)&si_other, 0, sizeof(si_other));
		si_other.sin_family = AF_INET;
		si_other.sin_port = htons(port);
		si_other.sin_addr.s_addr = inet_addr(address.c_str());

		int size_in = 512000;
		setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (char*)&size_in, sizeof(int));

		int result = ::bind(sock, (struct sockaddr *)&si_other, sizeof(si_other));
		if (result != 0) {
			std::cerr << "Could not bind to socket" << std::endl;

			return false;
		}

		// ???
		u_long mode = 0; // 0-> non bloccante, 1 -> bloccante
		ioctlsocket(sock, FIONBIO, &mode);

		return true;
	}

	// Send data to an address.
	int UDPSocket::sendTo(const std::string& address, unsigned short port, const char* buffer, int len, int flags) {
		sockaddr_in add;
		add.sin_family = AF_INET;
		add.sin_addr.s_addr = inet_addr(address.c_str());
		add.sin_port = htons(port);
		return sendto(sock, buffer, len, flags, reinterpret_cast<SOCKADDR *>(&add), sizeof(add));
	}

	// Receive data from an address.
	bool UDPSocket::receiveFrom(char* buffer, int &len, int flags) {
		sockaddr_in from;
		int size = sizeof(from);
		int ret = recvfrom(sock, buffer, len, flags, reinterpret_cast<SOCKADDR *>(&from), &size);

		if (ret < 0)
			//throw std::system_error(WSAGetLastError(), std::system_category(), "Recvfrom failed");
			return false;

		len = ret;
		return true;
	}
}
