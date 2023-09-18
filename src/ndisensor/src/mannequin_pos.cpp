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

bool fileExists(const std::string &Filename)
{
	return ACCESS(Filename.c_str(), 0) == 0;
}

std::string getToolInfo(std::string toolHandle)
{
	// Get the port handle info from PHINF
	PortHandleInfo info = capi.portHandleInfo(toolHandle);

	// Return the ID and SerialNumber the desired string format
	std::string outputString = info.getToolId();
	outputString.append(" s/n:").append(info.getSerialNumber());
	return outputString;
}

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

void loadTool(const char *toolDefinitionFilePath)
{
	// Request a port handle to load a passive tool into
	int portHandle = capi.portHandleRequest();
	onErrorPrintDebugMessage("capi.portHandleRequest()", portHandle);

	// Load the .rom file using the previously obtained port handle
	capi.loadSromToPort(toolDefinitionFilePath, portHandle);
}

std::string toolDataToCSV(const ToolData &toolData, ros::Publisher new_marker_pub)
{
	std::stringstream stream;
	stream << std::setprecision(toolData.PRECISION) << std::setfill('0');
	stream << "" << static_cast<unsigned>(toolData.frameNumber) << ","
		   << "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ",";
	stream << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ",";

	std_msgs::Float64MultiArray msg;

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
		msg.data.push_back(toolData.transform.tx);
		msg.data.push_back(toolData.transform.ty);
		msg.data.push_back(toolData.transform.tz);
		msg.data.push_back(toolData.transform.q0);
		msg.data.push_back(toolData.transform.qx);
		msg.data.push_back(toolData.transform.qy);
		msg.data.push_back(toolData.transform.qz);
		new_marker_pub.publish(msg);
	}

	// Each marker is printed as: status,tx,ty,tz
	// stream << "," << toolData.markers.size();
	// for (int i = 0; i < toolData.markers.size(); i++)
	// {
	// 	stream << "," << MarkerStatus::toString(toolData.markers[i].status);
	// 	if (toolData.markers[i].status == MarkerStatus::Missing)
	// 	{
	// 		stream << ",,,";
	// 	}
	// 	else
	// 	{
	// 		stream << "," << toolData.markers[i].x << "," << toolData.markers[i].y << "," << toolData.markers[i].z;
	// 	}
	// }
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

	// capi.portHandleRequest("********","*","1","**","01");
	// capi.portHandleFree("01");
	// capi.portHandleInfo("00");

	// capi.portHandleInitialize("01");
	// capi.portHandleEnable("01");
	// capi.loadSromToPort("/home/project/Downloads/Mannequin1.rom", 01);
	// capi.setUserParameter("Param.Tracking.Selected Volume","1");

	std::cout << std::endl
			  << "Entering tracking mode..." << std::endl;
	onErrorPrintDebugMessage("capi.startTracking()", capi.startTracking());

	while (ros::ok())
	{
		// Specifying tracking needed for strays
		std::vector<ToolData> toolData = capi.getTrackingDataBX2();
		printToolData(toolData[0], markerpos_pub);
		ros::spinOnce();
		loop_rate.sleep();
	}

	// Stop tracking (back to configuration mode)
	std::cout << std::endl
			  << "Leaving tracking mode and returning to configuration mode..." << std::endl;
	onErrorPrintDebugMessage("capi.stopTracking()", capi.stopTracking());

	std::cout << "Stray marker publishing complete." << std::endl;
	return 0;
}