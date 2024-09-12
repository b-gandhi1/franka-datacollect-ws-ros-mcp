#define ACCESS access
#include <ros/ros.h>
#include <unistd.h> // for POSIX sleep(sec), and access()
#include <sys/ioctl.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>

#include <polaris_pkg/CombinedApi.h>
#include <polaris_pkg/PortHandleInfo.h>
#include <polaris_pkg/ToolData.h>
#include <std_msgs/Float64MultiArray.h>
#include "geometry_msgs/PoseStamped.h"

static CombinedApi capi = CombinedApi();
static bool apiSupportsBX2 = true;
static bool apiSupportsStreaming = true;
double restl[4][1];

/*
* Conversion factors.
*/
#define RAD_TO_DEGREES (180 / 3.1415926)
/*
* Defined data types.
*/
typedef float
RotationMatrix[3][3];

typedef struct Rotation
{
	float
		fRoll,	/* rotation about the object's z-axis (Euler angle) */
		fPitch, /* rotation about the object's y-axis (Euler angle) */
		fYaw;	/* rotation about the object's x-axis (Euler angle) */
} Rotation;
typedef struct QuatRotation
{
	float
		fQ0,
		fQX,
		fQY,
		fQZ;
} QuatRotation;

void sleepSeconds(unsigned numSeconds)
{
	sleep(numSeconds); // sleep(sec)
}

/**
 * @brief Prints a debug message if a method call failed.
 * @details To use, pass the method name and the error code returned by the method.
 *          Eg: onErrorPrintDebugMessage("capi.initialize()", capi.initialize());
 *          If the call succeeds, this method does nothing.
 *          If the call fails, this method prints an error message to stdout.
 */
void onErrorPrintDebugMessage(std::string methodName, int errorCode)
{
	if (errorCode < 0)
	{
		std::cout << methodName << " failed: " << capi.errorToString(errorCode) << std::endl;
	}
}

// To check if the .rom file provided as command line argument exists
bool fileExists(const std::string &Filename)
{
	return ACCESS(Filename.c_str(), 0) == 0;
}

/**
 * @brief Returns the string: "[tool.id] s/n:[tool.serialNumber]" used in CSV output
 */
std::string getToolInfo(std::string toolHandle)
{
	// Get the port handle info from PHINF
	PortHandleInfo info = capi.portHandleInfo(toolHandle);

	// Return the ID and SerialNumber the desired string format
	std::string outputString = info.getToolId();
	outputString.append(" s/n:").append(info.getSerialNumber());
	return outputString;
}

/**
 * @brief Initialize and enable loaded tools. This is the same regardless of tool type.
 */
void initializeAndEnableTools(std::vector<ToolData> &enabledTools)
{
	std::cout << std::endl
			  << "Initializing and enabling tools..." << std::endl;

	// Initialize and enable tools
	std::vector<PortHandleInfo> portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
	for (int i = 0; i < portHandles.size(); i++)
	{
		onErrorPrintDebugMessage("capi.portHandleInitialize()", capi.portHandleInitialize(portHandles[i].getPortHandle()));
		onErrorPrintDebugMessage("capi.portHandleEnable()", capi.portHandleEnable(portHandles[i].getPortHandle()));
	}

	// Print all enabled tools
	portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
	for (int i = 0; i < portHandles.size(); i++)
	{
		std::cout << portHandles[i].toString() << std::endl;
	}

	// Lookup and store the serial number for each enabled tool
	for (int i = 0; i < portHandles.size(); i++)
	{
		enabledTools.push_back(ToolData());
		enabledTools.back().transform.toolHandle = (uint16_t)capi.stringToInt(portHandles[i].getPortHandle());
		enabledTools.back().toolInfo = getToolInfo(portHandles[i].getPortHandle());
	}
}

/**
 * @brief Loads a tool from a tool definition file (.rom)
 */
void loadTool(const char *toolDefinitionFilePath)
{
	// Request a port handle to load a passive tool into
	int portHandle = capi.portHandleRequest();
	onErrorPrintDebugMessage("capi.portHandleRequest()", portHandle);

	// Load the .rom file using the previously obtained port handle
	capi.loadSromToPort(toolDefinitionFilePath, portHandle);
}
void transf_func(double joint_mat[4][1])
{
	double tx = 15;
	double ty = 100;
	double tz = -1000;

	double t[4][4] = {{1, 0, 0, tx}, {0, 1, 0, ty}, {0, 0, 1, tz}, {0, 0, 0, 1}};

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 1; j++)
		{
			restl[i][j] = 0;

			for (int k = 0; k < 4; k++)
			{
				restl[i][j] += t[i][k] * joint_mat[k][j];
			}
		}
	}
}

void DetermineEuler(RotationMatrix dtRotMatrix, Rotation *pdtEulerRot)
{
	float
		fRoll,
		fCosRoll,
		fSinRoll;
	fRoll = atan2(dtRotMatrix[1][0], dtRotMatrix[0][0]);
	fCosRoll = cos(fRoll);
	fSinRoll = sin(fRoll);
	pdtEulerRot->fRoll = fRoll;
	pdtEulerRot->fPitch = atan2(-dtRotMatrix[2][0],
								(fCosRoll * dtRotMatrix[0][0]) + (fSinRoll *
																  dtRotMatrix[1][0]));
	pdtEulerRot->fYaw = atan2(
		(fSinRoll * dtRotMatrix[0][2]) -
			(fCosRoll * dtRotMatrix[1][2]),
		(-fSinRoll * dtRotMatrix[0][1]) +
			(fCosRoll * dtRotMatrix[1][1]));
} /* DetermineEuler */

void CvtQuatToRotationMatrix(QuatRotation *pdtQuatRot, RotationMatrix dtRotMatrix)
{
	float
		fQ0Q0,
		fQxQx,
		fQyQy,
		fQzQz,
		fQ0Qx,
		fQ0Qy,
		fQ0Qz,
		fQxQy,
		fQxQz,
		fQyQz;
	/*
	 * Determine some calculations done more than once.
	 */
	fQ0Q0 = pdtQuatRot->fQ0 * pdtQuatRot->fQ0;
	fQxQx = pdtQuatRot->fQX * pdtQuatRot->fQX;
	fQyQy = pdtQuatRot->fQY * pdtQuatRot->fQY;
	fQzQz = pdtQuatRot->fQZ * pdtQuatRot->fQZ;
	fQ0Qx = pdtQuatRot->fQ0 * pdtQuatRot->fQX;
	fQ0Qy = pdtQuatRot->fQ0 * pdtQuatRot->fQY;
	fQ0Qz = pdtQuatRot->fQ0 * pdtQuatRot->fQZ;
	fQxQy = pdtQuatRot->fQX * pdtQuatRot->fQY;
	fQxQz = pdtQuatRot->fQX * pdtQuatRot->fQZ;
	fQyQz = pdtQuatRot->fQY * pdtQuatRot->fQZ;
	/*
	 * Determine the rotation matrix elements.
	 */
	dtRotMatrix[0][0] = fQ0Q0 + fQxQx - fQyQy - fQzQz;
	dtRotMatrix[0][1] = 2.0 * (-fQ0Qz + fQxQy);
	dtRotMatrix[0][2] = 2.0 * (fQ0Qy + fQxQz);
	dtRotMatrix[1][0] = 2.0 * (fQ0Qz + fQxQy);
	dtRotMatrix[1][1] = fQ0Q0 - fQxQx + fQyQy - fQzQz;
	dtRotMatrix[1][2] = 2.0 * (-fQ0Qx + fQyQz);
	dtRotMatrix[2][0] = 2.0 * (-fQ0Qy + fQxQz);
	dtRotMatrix[2][1] = 2.0 * (fQ0Qx + fQyQz);
	dtRotMatrix[2][2] = fQ0Q0 - fQxQx - fQyQy + fQzQz;
} /* CvtQuatToRotationMatrix */

Rotation CvtQuatToEulerRotation(QuatRotation *pdtQuatRot)
{
	Rotation *pdtEulerRot;
	RotationMatrix dtRotMatrix;
	CvtQuatToRotationMatrix(pdtQuatRot, dtRotMatrix);
	DetermineEuler(dtRotMatrix, pdtEulerRot);
	pdtEulerRot->fYaw *= RAD_TO_DEGREES;
	pdtEulerRot->fPitch *= RAD_TO_DEGREES;
	pdtEulerRot->fRoll *= RAD_TO_DEGREES;

	return *pdtEulerRot;
} /* CvtQuatToEulerRotation */

/**
 * @brief Function to retrieve the data and publish it to the topic
 *
 * @param toolData
 * @param new_marker_pub
 * @return Pose of the tool detected
 */

std::string toolDataPublisher(const ToolData &toolData, ros::Publisher new_marker_pub)
{
	std::stringstream stream;
	stream << std::setprecision(toolData.PRECISION) << std::setfill('0');
	stream << "" << static_cast<unsigned>(toolData.frameNumber) << ","
		   << "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ",";
	stream << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ",";

	geometry_msgs::PoseStamped msg;

	if (toolData.transform.isMissing())
	{
		stream << "Missing,,,,,,,,";
	}
	else
	{
		stream << TransformStatus::toString(toolData.transform.getErrorCode()) << ","
			   << toolData.transform.q0 << "," << toolData.transform.qx << "," << toolData.transform.qy << "," << toolData.transform.qz << ","
			   << toolData.transform.tx << "," << toolData.transform.ty << "," << toolData.transform.tz << "," << toolData.transform.error;

		// Publish Data
		// double pos_mat[4][1] = {{toolData.transform.tx},{toolData.transform.ty},{toolData.transform.tz},(1.0)};
		// transf_func(pos_mat);
		
		// convert to euler: 
		Rotation rotarray;

		QuatRotation samp_rot;
		samp_rot.fQ0 = toolData.transform.q0;
		samp_rot.fQX = toolData.transform.qx;
		samp_rot.fQY = toolData.transform.qy;
		samp_rot.fQZ = toolData.transform.qz;

		rotarray = CvtQuatToEulerRotation(&samp_rot);

		msg.pose.position.x = toolData.transform.tx;
		msg.pose.position.y = toolData.transform.ty;
		msg.pose.position.z = toolData.transform.tz;
		// msg.pose.orientation.w = toolData.transform.q0;
		msg.pose.orientation.x = rotarray.fRoll;
		msg.pose.orientation.y = rotarray.fPitch;
		msg.pose.orientation.z = rotarray.fYaw;	

		new_marker_pub.publish(msg);
	}
	return stream.str();
}

/**
 * @brief Prints a ToolData object to stdout
 * @param toolData The data to print
 * @param marker_pub The ROS publisher name
 * @param marker_pub2
 */

void printToolData(const ToolData &toolData, ros::Publisher marker_pub)
{
	std::cout << std::endl
			  << "Entered Print tool data function"
			  << "\n";

	// Detects any alerts for e.g. low temperature, and prints them to the screen
	if (toolData.systemAlerts.size() > 0)
	{
		std::cout << "[" << toolData.systemAlerts.size() << " alerts] ";
		for (int a = 0; a < toolData.systemAlerts.size(); a++)
		{
			std::cout << toolData.systemAlerts[a].toString() << std::endl;
		}
	}

	if (toolData.buttons.size() > 0)
	{
		std::cout << "[buttons: ";
		for (int b = 0; b < toolData.buttons.size(); b++)
		{
			std::cout << ButtonState::toString(toolData.buttons[b]) << " ";
		}
		std::cout << "] ";
	}
	std::cout << toolDataPublisher(toolData, marker_pub) << std::endl;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "marker_position");
	ros::NodeHandle n;

	// ROS Publisher
	ros::Publisher markerpos_pub = n.advertise<geometry_msgs::PoseStamped>("Marker_Pos", 1000);
	ros::Publisher markerpos_pub2 = n.advertise<geometry_msgs::PoseStamped>("Marker_Pos2", 1000);
	ros::Rate loop_rate(100);

	// Validate the number of arguments received
	if (argc < 2 || argc > 4)
	{
		std::cout << "CAPIsample Ver " << capi.getVersion() << std::endl
				  << "usage: ./capisample <hostname> [args]" << std::endl
				  << "where:" << std::endl
				  << "    <hostname>      (required) The measurement device's hostname, IP address, or serial port." << std::endl
				  << "    [args]          (optional) Any other arguments such as tools to load, or SCU to connect to." << std::endl
				  << "example hostnames:" << std::endl
				  << "    Connecting to device by IP address: 169.254.8.50" << std::endl
				  << "    Connecting to device by hostname: P9-B0103.local" << std::endl
				  << "    Connecting to serial port varies by operating system:" << std::endl
				  << "        COM10 (Windows), /dev/ttyUSB0 (Linux), /dev/cu.usbserial-001014FA (Mac)" << std::endl
				  << "Optional arguments:" << std::endl
				  << "--scu=[scu_hostname] A System Control Unit (SCU) hostname, used to connect active tools." << std::endl
				  << "--stream=UDP Specify the streaming protocol as UDP (default is TCP)" << std::endl
				  << "--tools=[file1.rom],[file2.rom]... A comma delimited list of tools to load." << std::endl;
		return -1;
	}

	// Ignore argv[0], and assign the hostname as required
	std::string hostname = std::string(argv[1]);

	// Look for optional arguments
	std::vector<std::string> toolDefinitions = std::vector<std::string>();
	for (int i = 2; i < argc; i++)
	{
		std::string arg(argv[i]);
		if (arg.substr(0, 8).compare("--tools=") == 0)
		{
			std::istringstream sstream(arg.substr(8));
			std::string tool;
			while (getline(sstream, tool, ','))
			{
				// If the file is accessible, add it to the list of tools to load
				if (fileExists(tool))
				{
					toolDefinitions.push_back(tool);
				}
				else
				{
					std::cerr << "Cannot access file: " << tool << std::endl;
				}
			}
		}
	}

	// Attempt to connect to the device
	if (capi.connect(hostname) != 0)
	{
		// Print the error and exit if we can't connect to a device
		std::cout << "Connection Failed!" << std::endl;
		std::cout << "Press Enter to continue...";
		std::cin.ignore();
		return -1;
	}
	std::cout << "Connected!" << std::endl;

	// Wait a second - needed to support connecting to LEMO Vega
	sleepSeconds(1);

	// Initialize the system. This clears all previously loaded tools, unsaved settings etc...
	onErrorPrintDebugMessage("capi.initialize()", capi.initialize());

	// Load any passive tool definitions from a .rom files
	if (toolDefinitions.size() > 0)
	{
		std::cout << "Loading Tool Definitions (.rom files) ..." << std::endl;
		for (int f = 0; f < toolDefinitions.size(); f++)
		{
			std::cout << "Loading: " << toolDefinitions[f] << std::endl;
			loadTool(toolDefinitions[f].c_str());
		}
	}

	// Once loaded or detected, tools are initialized and enabled the same way
	// PHSR can be time consuming, so store the tool metadata immediately (eg. port handle, serial number)
	std::vector<ToolData> enabledTools = std::vector<ToolData>();
	initializeAndEnableTools(enabledTools);

	std::cout << std::endl
			  << "Entering tracking mode..." << std::endl;
	onErrorPrintDebugMessage("capi.startTracking()", capi.startTracking());

	while (ros::ok())
	{
		// Specifying tracking needed for strays
		std::vector<ToolData> toolData = capi.getTrackingDataBX2(); //  test what works well: empty OR "--6d=tools --3d=tools --sensor=none --1d=none"
		printToolData(toolData[0], markerpos_pub);
		printToolData(toolData[1], markerpos_pub2);
		ros::spinOnce();
		loop_rate.sleep();
	}

	// Stop tracking (back to configuration mode)
	std::cout << std::endl
			  << "Leaving tracking mode and returning to configuration mode..." << std::endl;
	onErrorPrintDebugMessage("capi.stopTracking()", capi.stopTracking());

	// exit
	std::cout << "Marker publishing complete." << std::endl;
	return 0;
}