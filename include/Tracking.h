#ifndef __ARMD_API_H_
#define __ARMD_API_H_

#include "UdpReader.h"
#include "ToolRegi.h"
#include "SystemCRC.h"

/**
 * @brief This class contains methods to exchange datas from ARMD RUITONG optical racking device (SE/MAX/LITE)
 */

namespace TrackingReplyOption
{
	//! The reply options used by BX and TX commands
	enum value { TransformData = 0x0001, ToolAndMarkerData = 0x0002, SingleStray3D = 0x0004, Tool3Ds = 0x0008, AllTransforms = 0x0800, PassiveStrays = 0x1000 };
}

class Tracking
{
public:
	Tracking(std::string hostname);
	~Tracking();

	//get connection IP
	std::vector<std::string> GetIPs();

	//clear buffer
	void ClearBuffer();

	//clear last command to initialize
	int Initialize();

	//get parameter
	std::vector<double> getParameter();

	//start and end tracking
	int StartTraking();
	int EndTracking();

	//start and end imaging
	int StartImaging();
	int EndImaging();
	
	//get all markers in the screen
	std::vector<MarkerPosition> GetCordi() { return this->Cordis; };

	//update tracking information
	int TrackingBX(std::vector<std::string>& warnMessage, std::vector<MarkerPosition>& Cordis, char* leftimg, char* rightimg);

	//
	std::vector<int> readFirmwareVersion();

	//send command to device
	int sendCommand(std::string command) const;
	int sendAN5642Command(char* command) const;

	//check ingormation
	void SystemCheck(std::string& warnMsg);
	void MessageCheck(std::string& msg);

	//close connection
	int CloseUDP();

private:
	UdpConnection* connection_;
	UdpReader reader;
	SystemCRC* crcValidator_;
	std::vector<MarkerPosition> Cordis;

	// void ProcBX(UdpReader& reader, std::vector<MarkerData>& Cordis);
};

#endif __ARMD_API_H_