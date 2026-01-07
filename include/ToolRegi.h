#ifndef TOOLREGI_HPP
#define TOOLREGI_HPP

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>

/**
 * @brief This class contains methods to process the tracking information from ARMD RUITONG optical racking device (SE/MAX/LITE)
 */

namespace TransformationStatus
{
	enum values
	{
		Enabled = 0x0000,
		PartiallyOutOfVolume = 0x0001,
		OutOfVolume = 0x0002,
		PartiallyMatched = 0x0003,
		TooFewMarkers = 0x0004,
		BadTransformFit = 0x0005,
		// = 0x0006,
		// = 0x0007,
		// = 0x0008,
		ToolMissing = 0x0009,
	};

	// converts the error code to std::string representation
	std::string toString(uint8_t errorCode);
}

struct ToolPlaneMarker
{
	std::vector<std::vector<double>> regiSet;
	std::vector<std::vector<double>> regiDifSet;
};

struct MarkerPosition
{
	double P[4];//marker position (x,y,z,1)
	int leftExposure = 0;//marker exposure
	int rightExposure = 0;//marker exposure
};

struct Transformation
{
	uint16_t status = TransformationStatus::ToolMissing;//transformation status
	double matrix[4][4];//transformation matrix [R|t]
	double qw, qx, qy, qz;//transformation quaternion
	double tx, ty, tz;//tip position
	double error;//matching error
};

struct ToolTrackingData
{
	std::string name = "";//tool name
	Transformation transform;//transform information
	int matchStatus = 0;//matching status
	int matchedPlane = 0;//matched plane index
	int matchedMarkersNum = 0;//matched markers number on the plane
	std::vector<MarkerPosition> markers;//positions of all markers on the matched plane
	std::string timespec = "";//acquisition time
};

class ToolRegi
{
public:
	ToolRegi(int version);
	~ToolRegi();

	//input the tool to be tracked
	void SetInputData(std::vector<std::vector<MarkerPosition>> source, std::vector<MarkerPosition> target, std::vector<ToolPlaneMarker> planeMarkers, double tip[3], int minMatchingNum, double maxFRE);
	
	//match markers to tool
	bool Register();
	
	//get tool tracking data
	void GetTrackingData(ToolTrackingData& data);

protected:
	int deviceVersion = 0;

	ToolTrackingData m_Data;
	uint8_t status;
	std::vector<std::vector<MarkerPosition>> toolMarker;
	std::vector<MarkerPosition> targetMarker;
	double tipPoint[3];
	double RegMatrix[4][4];
	double FRE;
	std::vector<int> order;

	int m_MinRegNum = 3;
	double m_MaxFRE = 1;
	std::vector<ToolPlaneMarker> toolPlaneMarker;

	std::vector<MarkerPosition> ArrangeTri(std::vector<MarkerPosition> src, bool record, std::vector<double> set);
	bool TriMatch(std::vector<MarkerPosition> a, std::vector<MarkerPosition> b);
	void Mat2Quat(double mat[4][4], Transformation& tran);
	std::vector<MarkerPosition> ReOrderMarkers(std::vector<MarkerPosition>, std::vector<double> set, std::vector<double> difset, int planeIndex);

	void PointRegister(std::vector<MarkerPosition> source, std::vector<MarkerPosition> target, double Matrix[4][4]);
	int JacobiN(double** a, int n, double* w, double** v);
	void Perpendiculars(const double v1[3], double v2[3], double v3[3], double theta);
	double GetFRE();
	static double EucDist(double* a, double* b);
	MarkerPosition MatrixPlusPoint(double Matrix[4][4], MarkerPosition point);

	float getCosine(std::vector<float> Vector1, std::vector<float> Vector2);
	std::vector<float> getVector(MarkerPosition p1, MarkerPosition p2);
	float getDis(MarkerPosition p1, MarkerPosition p2);
	std::vector<float> use3PointsComputeNormalVector(MarkerPosition p1, MarkerPosition p2, MarkerPosition p3);
	MarkerPosition getProjectionPt(MarkerPosition p, MarkerPosition p1, MarkerPosition p2, MarkerPosition p3);
	int judgePtInOut(MarkerPosition p, std::vector<MarkerPosition> Pts);
	int judgePtInOut_SE(std::vector<MarkerPosition> Pts);
	int judgePtInOut_MAX(std::vector<MarkerPosition> Pts);
	std::vector<MarkerPosition> loadVertexSE();
	std::vector<MarkerPosition> loadVertexMAX();
	void VersionDetection(std::string hostname);
};

#endif