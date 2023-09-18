#define ACCESS access
#include <unistd.h>	 // for POSIX sleep(sec), and access()
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

static CombinedApi capi = CombinedApi();
static bool apiSupportsBX2 = false;
static bool apiSupportsStreaming = false;

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

std::string getToolInfo(std::string toolHandle)
{
	// Get the port handle info from PHINF
	PortHandleInfo info = capi.portHandleInfo(toolHandle);

	// Return the ID and SerialNumber the desired string format
	std::string outputString = info.getToolId();
	outputString.append(" s/n:").append(info.getSerialNumber());
	return outputString;
} 


void determineApiSupportForBX2()
{
	// Lookup the API revision
	std::string response = capi.getApiRevision();
	
	// Refer to the API guide for how to interpret the APIREV response
	char deviceFamily = response[0];
	int majorVersion = capi.stringToInt(response.substr(2,3));

	// As of early 2017, the only NDI device supporting BX2 is the Vega
	// Vega is a Polaris device with API major version 003
	if ( deviceFamily == 'G' && majorVersion >= 3)
	{
		apiSupportsBX2 = true;
        apiSupportsStreaming = true;
	}
}

std::string toolDataToCSV(const ToolData& toolData)
{
	std::cout << std::endl <<"Entered toolDatatoCSV function"<<"\n";
	std::stringstream stream;
	stream << std::setprecision(toolData.PRECISION) << std::setfill('0');
	stream << "" << static_cast<unsigned>(toolData.frameNumber) << ","
		   << "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ",";
	stream << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ",";

	if (toolData.transform.isMissing())
	{
		stream << " Transform Missing,,,,,,,,";
	}
	else
	{
		stream << TransformStatus::toString(toolData.transform.getErrorCode()) << ","
			   << toolData.transform.q0 << "," << toolData.transform.qx << "," << toolData.transform.qy << "," << toolData.transform.qz << ","
			   << toolData.transform.tx << "," << toolData.transform.ty << "," << toolData.transform.tz << "," << toolData.transform.error;
	}

	// Each marker is printed as: status,tx,ty,tz
	stream << "," << toolData.markers.size();
	for ( int i = 0; i < toolData.markers.size(); i++)
	{
		stream << "," << MarkerStatus::toString(toolData.markers[i].status);
		if (toolData.markers[i].status == MarkerStatus::Missing)
		{
			stream << ",,,";
		}
		else
		{
			stream << "," << toolData.markers[i].x << "," << toolData.markers[i].y << "," << toolData.markers[i].z;
		}
	}
	return stream.str();
}

void printToolData(const ToolData& toolData)
{
	std::cout << std::endl <<"Entered Print tool data function"<<"\n";
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
	std::cout << toolDataToCSV(toolData) << std::endl;
}

void printTrackingData()
{
	// Start tracking, output a few frames of data, and stop tracking

	for (int i = 0; i < 10; i++)
	{
		std::cout <<"Entered Print tracking function"<<"\n";
		// Demonstrate TX command: ASCII command sent, ASCII reply received
		//std::cout << capi.getTrackingDataTX(1000) << std::endl;
		
		// Demonstrate BX or BX2 command
		std::vector<ToolData> toolData =  capi.getTrackingDataBX2("--3d=strays");

		// Print to stdout in similar format to CSV
		//std::cout << "[alerts] [buttons] Frame#,ToolHandle,Face#,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers,State,Tx,Ty,Tz" << std::endl;
		//std::cout <<toolData.data()<<"\n";
		for (int i = 0; i < toolData.size(); i++)
		{
			//std::cout << toolData[i].transform.tx;
			printToolData(toolData[i]);
		} 
	}
}

int main(int argc, char* argv[])
{
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

	// Print the firmware version for debugging purposes
	std::cout << capi.getUserParameter("Features.Firmware.Version") << std::endl;

	// Determine if the connected device supports the BX2 command
	determineApiSupportForBX2();

    if (apiSupportsBX2 && apiSupportsStreaming)
    {
        std::cout<<"BX2 is supported"<<"\n";
    }
    
	// Initialize the system. This clears all previously loaded tools, unsaved settings etc...
	onErrorPrintDebugMessage("capi.initialize()", capi.initialize());

	capi.portHandleRequest("********","*","1","**","01");
	capi.portHandleInitialize("01");
	capi.portHandleEnable("01"); 

	/* std::vector<ToolData> enabledTools = std::vector<ToolData>();
    initializeAndEnableTools(enabledTools);

    // Print an error if no tools were specified
    if (enabledTools.size() == 0)
    {
        std::cout << "No tools detected. To load passive tools, specify: --tools=[tool1.rom],[tool2.rom]" << std::endl;
    } */ 

	// Once the system is put into tracking mode, data is returned for whatever tools are enabled
    std::cout << std::endl << "Entering tracking mode..." << std::endl;
    onErrorPrintDebugMessage("capi.startTracking()", capi.startTracking());

    if (apiSupportsStreaming)
    {
        // Demonstrate polling a few frames of data
        printTrackingData();
		
        // Write a CSV file
        //writeCSV("example.csv", 50, enabledTools);
    }
    
    // Stop tracking (back to configuration mode)
    std::cout << std::endl << "Leaving tracking mode and returning to configuration mode..." << std::endl;
    onErrorPrintDebugMessage("capi.stopTracking()", capi.stopTracking());

    std::cout << "CAPI demonstration complete. Press any key to exit." << std::endl;
    return 0; 
}