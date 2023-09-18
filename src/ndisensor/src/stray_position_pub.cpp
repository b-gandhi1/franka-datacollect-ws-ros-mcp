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

#include <ndisensor/CombinedApi.h>
#include <ndisensor/PortHandleInfo.h>
#include <ndisensor/ToolData.h>
#include <std_msgs/Float64MultiArray.h>

static CombinedApi capi = CombinedApi();
static bool apiSupportsBX2 = true;
static bool apiSupportsStreaming = true;

void sleepSeconds(unsigned numSeconds)
{
	sleep(numSeconds); // sleep(sec)
}

void onErrorPrintDebugMessage(std::string methodName, int errorCode)
{
	if (errorCode < 0)
	{
		std::cout << methodName << " failed: " << capi.errorToString(errorCode) << std::endl;
	}
}

std::string toolDataToCSV(const ToolData &toolData, ros::Publisher new_marker_pub)
{
	std::cout << std::endl
			  << "Entered toolDatatoCSV function"
			  << "\n";
	std::stringstream stream;
	stream << std::setprecision(toolData.PRECISION) << std::setfill('0');
	stream << "" << static_cast<unsigned>(toolData.frameNumber) << ","
		   << "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ",";
	stream << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ",";

	// Each marker is printed as: status,tx,ty,tz
	stream << "," << toolData.markers.size() << "\n";

	std::array<std::string, 4> jnt_names = {"left_wrist", "left_elbow", "left_shoulder", "chest"};
	std_msgs::Float64MultiArray msg;
	for (int i = 0; i < toolData.markers.size(); i++)
	{
		stream << "," << MarkerStatus::toString(toolData.markers[i].status) << "->";
		if (toolData.markers[i].status == MarkerStatus::Missing)
		{
			stream << ",,,";
		}
		else
		{
			stream << jnt_names[i] << ": " << toolData.markers[i].x << "," << toolData.markers[i].y << "," << toolData.markers[i].z << "\n";
			msg.data.push_back(toolData.markers[i].x);
			msg.data.push_back(toolData.markers[i].y);
			msg.data.push_back(toolData.markers[i].z);
			new_marker_pub.publish(msg);
		}
	}

	return stream.str();
}

void printToolData(const ToolData &toolData, ros::Publisher marker_pub)
{
	std::cout << std::endl
			  << "Entered Print tool data function"
			  << "\n";
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
	std::cout << toolDataToCSV(toolData, marker_pub) << std::endl;
	// ros::Duration(0.5).sleep();
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "marker_position");
	ros::NodeHandle n;

	// ROS Publisher
	ros::Publisher markerpos_pub = n.advertise<std_msgs::Float64MultiArray>("Marker_Pos", 1000);
	ros::Rate loop_rate(10);

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

	capi.portHandleRequest("********", "*", "1", "**", "01");
	capi.portHandleInitialize("01");
	capi.portHandleEnable("01");

	capi.getUserParameter("Features.Volumes.Index");
	capi.setUserParameter("Param.Video Camera.Allow Streaming", "1");

	std::cout << std::endl
			  << "Entering tracking mode..." << std::endl;
	onErrorPrintDebugMessage("capi.startTracking()", capi.startTracking());

	while (ros::ok())
	{
		// Specifying tracking needed for strays
		std::vector<ToolData> toolData = capi.getTrackingDataBX2("--3d=strays");
		printToolData(toolData[1], markerpos_pub);
	}

	// Stop tracking (back to configuration mode)
	std::cout << std::endl
			  << "Leaving tracking mode and returning to configuration mode..." << std::endl;
	onErrorPrintDebugMessage("capi.stopTracking()", capi.stopTracking());

	std::cout << "Stray marker publishing complete." << std::endl;
	return 0;
}