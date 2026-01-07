#ifndef ARMDCOMBINEDAPI_H
#define ARMDCOMBINEDAPI_H

#include "Tracking.h"

#include <chrono>
#include <iomanip>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>

/**
 * @brief This class contains methods to link and control ARMD RUITONG optical racking device (SE/MAX/LITE)
 */

namespace GenerationStatus
{
	enum value
	{
		OK = 0x0000,
		MinMatchedNumLess = 0x0001,
		MinMatchedNumMore = 0x0002,
		PlaneNumLess = 0x0003,
		PlaneNumMore = 0x0004,
		PlaneNumNotMatching = 0x0005,
		MarkerNumLess = 0x0006,
		MarkerNumMore = 0x0007,
		DistanceTooSmall = 0x0008,
		NameError = 0x0009,
		PathNotExist = 0x000A,
	};

	//return the std::string representation of the AROM generation status
	std::string toString(uint16_t generationCode);
}

namespace TransmissionStatus
{
	enum value
	{
		NoneData = 0x0000,
		TrackingData = 0x0001,
		ImagingData = 0x0002,
		AllData = 0x0003,
	};
}

namespace ConnectionStatus
{
	enum value
	{
		Connected = 0x0000,
		DisConnected = 0x0001,
		DisConnectedFailed = 0x0002,
		PortConflicts = 0x0003,
		ResloveError = 0x0004,
		InitialUDPError = 0x0005,
		Interruption = 0x0006,
		InvalidAddr = 0x0007,
		DeviceInexist = 0x0008,
		UnknownError = 0x0009,
	};

	//return the std::string representation of the connection status
	std::string toString(uint16_t conditionCode);
}

namespace DeviceAlert
{
	enum value
	{
		Normal = 0x0000,
		Vibration = 0x0001,
		TempTooHigh = 0x0002,
		TempTooLow = 0x0003,
	};

	//return the std::string representation of the system alert
	std::string toString(uint16_t alertCode);
}

struct ToolCalibrationData
{
	std::string name = "";//tool name
	bool type = false;//passive(0) or active(1) tool
	int minNumMarker = 3;//minmum number of markers to be matched
	int planeNum = 1;//plane
	std::vector<std::vector<MarkerPosition>> markers;//maker position per plane
	double pin[3] = {0.0, 0.0, 0.0};//tip position
	double calbError = -1;//calibration error
	double maxFRE = 1;//maximum matching FRE
	double maxAngle = 60;//maximum angle for matching
};

class ARMDCombinedAPI
{
public:
	ARMDCombinedAPI();
	virtual ~ARMDCombinedAPI();

	//connect RT device
	int connect(std::string hostname);

	//disconnect RT device
	void disconnect();

	//get local and device IP addresses
	std::vector<std::string> getConnectionIPs();

	//print API and firmware version
	void printAPIanfFirmwareVersion();

	//load tool information
	void loadPassiveToolAROM(std::string path, std::string parm = "cover");
	void loadActiveWirelessToolAROM(std::string path, std::string parm = "cover");
	void loadActiveToolAROM(std::string path, std::string parm = "cover");

	//get tool information
	std::vector<ToolCalibrationData> getToolStorage() { return this->trackTools; };
	int getTrackToolsNum() { return this->trackTools.size(); };

	//start and stop tracking
	void startTracking();
	void stopTracking();

	//start and stop imaging
	void startImaging();
	void stopImaging();

	//update tracking and imaging data
	void trackingUpdate();

	//get current tracking data
	std::vector<ToolTrackingData> getTrackingData(std::vector<MarkerPosition> cordi);
	std::vector<MarkerPosition> getAllMarkers() { return this->allMarkersTrackingData; };
	std::vector<MarkerPosition> getUnMatchedMarkers() { return this->unMatchedMarkers; };

	//get current imaging data
	char* getLeftImagingData();
	char* getRightImagingData();

	//get system time
	std::string getSystemTime();

	//get gravity vector
	std::vector<double> getGravityVector() {return this->gravity; };

	//get system status
	uint16_t getConnectionStatus() { return this->connectionStatus; };
	uint16_t getTransmissionStatus() { return this->transmissionStatus; };
	uint16_t getSystemAlert() { return this->systemAlert; };

	//create an arom file
	uint16_t generateAROM(std::string directory, ToolCalibrationData tool);

private:
	Tracking* TK;
	ToolRegi* ToolRegFilter;

	std::vector<int> version;
	std::vector<double> parameters;

	std::vector<ToolCalibrationData> trackTools;
	std::vector<std::vector<ToolPlaneMarker>> toolPlaneMarker;

	std::vector<MarkerPosition> unMatchedMarkers;
	std::vector<MarkerPosition> allMarkersTrackingData;

	char* leftimgDataforFrame;
	char* rightimgDataforFrame;

	uint16_t transmissionStatus;
	uint16_t connectionStatus;
	uint16_t systemAlert;

	std::vector<double> gravity{0.0, 0.0, 0.0};

	std::vector<ToolPlaneMarker> GenerateRegiSet(ToolCalibrationData tooldata);

	void getFiles(const char * dir_name, std::string extend_name, std::vector<std::string>&file_names);

	int toolReg(ToolCalibrationData tool, std::vector<ToolPlaneMarker> planeMarkers, std::vector<MarkerPosition> points);

	bool GetMarkersMinDis(std::vector<MarkerPosition> markerData);
	bool ExamineCharacter(std::string name);
};
#endif