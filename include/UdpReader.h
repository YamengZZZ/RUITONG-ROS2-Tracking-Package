#ifndef _UDP_READER_HPP_
#define _UDP_READER_HPP_

#include "UdpConnection.h"

/**
 * @brief This class contains methods to decode information from ARMD RUITONG optical racking device (SE/MAX/LITE)
 */

class UdpReader
{
public:
	UdpReader();
	UdpReader(UdpConnection* UdpCon);

    //reset and clear
	void reset();
	void clearBuffer();

	//get information
	int getSize();
	std::string getData(size_t start, size_t length) const;

	//process information
	int readBytes();
	void skipBytes(int numBytes);

	//transfer information
	byte_t get_byte();
	uint16_t get_uint16();
	uint32_t get_uint32();
	double get_double();

	//device's reply
	unsigned char buffer[1500];

private:
	UdpConnection* UdpCon_;
	
	int buffersize = 1500;
	int currentIndex_ = 0;
};
#endif
#pragma once
