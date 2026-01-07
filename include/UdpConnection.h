#ifndef UDP_CONNECTION_HPP
#define UDP_CONNECTION_HPP

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <netdb.h>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sstream>
#include <fstream>

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib, "iphlpapi.lib")
#define ASCII_Point 46
#define Num_Points 3

#include <stdint.h>
typedef uint8_t byte_t;

#define DEVICE_PORT 8080
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR,12)

/**
 * @brief This class contains methods to send and receive datas from ARMD RUITONG optical racking device (SE/MAX/LITE)
 */

class UdpConnection
{
public:
	UdpConnection(std::string hostname);
	~UdpConnection();

	//read and write information from device
	int read(byte_t* buffer, int length) const;
	int write(byte_t* buffer, int length) const;

	//clean buffer
	void cleanBuff();

	//get connection status
	int isConnected() const;

	//close UDP connection
	int closeUDP();

	//get connection IP
	std::vector<std::string> getConnectionIP();

	enum ConnectionError
	{
		nonError = 0,
		resloveError = -1,
		initialUDPError = -2,
		portConflicts = -3,
		invalidAddr = -4,
		deviceInexist = -5,
		unknownError = -10,
	};

private:
	//initialize UDP connection
	int init(std::string LocalIP, std::string DeviceIP);

	//configure device
	int ConfigureIpHostName(std::string input);

	//is or not IP
	bool IpDetection(std::string input);

	//get device and local IP
	std::string GetDevicIp(std::string hostname);
	std::string GetLocalIp(std::string Device_ipv4);
	int GetInfo(std::string text, std::vector<std::string>& Info);

	int isConnected_;
	int    deviceSocket_;
	struct sockaddr_in  addrSrv;
	struct sockaddr_in  addrLocal;

	std::string deviceIP;
	std::string localIP;
};

#endif
#pragma once
