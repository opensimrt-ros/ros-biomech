
/**
 *	This is the example from vicon. I havent touched it yet!!
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

#include <boost/system/error_code.hpp>
#include <opensimrt_msgs/SetFileNameSrv.h>
#include <std_srvs/Empty.h>

#include "ros/rate.h"
#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>

#include <string>
#include <time.h>

#pragma warning( push )
#pragma warning( disable : 4242 )
#include <boost/asio.hpp>
#pragma warning( pop )

bool start, stop;
std::string dirname;
std::string filename;
std::string description;
std::string notes;

bool startRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("startRecording service called.");
	start = true;
	stop = false;
	return true;
}
bool stopRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("stopRecording service called.");
	start = false;
	stop = true;
	return true;
}
bool setNamePath(opensimrt_msgs::SetFileNameSrv::Request &req, opensimrt_msgs::SetFileNameSrv::Response &res)
{
	ROS_INFO_STREAM("name " << req.name << " path:"<< req.path);

	std::string dirname = req.path;
	std::string filename = req.name;
	std::string description = req.description;
	std::string notes = req.notes;

	return true;
}



int main( int argc, char *argv[] )
{
  std::string CaptureName;
  std::string CaptureNotes;
  std::string CaptureDescription;
  std::string CaptureDatabasePath;
  std::string Timecode;
  unsigned int CaptureDelayInMilliseconds = 0;
  unsigned short Port = 30;

  // Parse out the options from the command line.
  // The name and path are mandatory for both the start and stop signals.

  boost::program_options::options_description Description("Allowed options");
  Description.add_options()
      ("help", "Show help message")
      ("start", "Send start signal")
      ("stop", "Send stop signal")

      ("name", boost::program_options::value< std::string >( &CaptureName )->required(), "The name trial to capture (required)" )
      ("notes", boost::program_options::value< std::string >( &CaptureNotes ), "The notes associated with the trial" )
      ("description", boost::program_options::value< std::string >( &CaptureDescription ), "The description of the trial" )
      ("path", boost::program_options::value< std::string >( &CaptureDatabasePath )->required(), "The database capture path (required)" )

      ("timecode", boost::program_options::value< std::string >( &Timecode ), "A timecode to start / stop on" )
  ;

  boost::program_options::variables_map Variables;

  try
  {
    boost::program_options::store( boost::program_options::parse_command_line(argc, argv, Description), Variables );
  }
  catch( boost::program_options::invalid_option_value Exception )
  {
    std::cout << Description << std::endl;
    return 1;
  }

  try
  {
    boost::program_options::notify( Variables );
  }
  catch( boost::program_options::required_option Exception )
  {
    std::cout << Description << std::endl;
    std::cout << "--name and --path are required." << std::endl;

    return 1;
  }

  if( Variables.count("help") ) 
  {
    std::cout << Description << std::endl;
    return 1;
  }

  // One of stop or start should have been specified

  if( Variables.count("start") + Variables.count("stop") != 1 ) 
  {
    std::cout << "Usage:" << std::endl;
    std::cout << argv[0] << " --start --name Test --description \"Captured automatically.\" --path \"C:/Temp\"" << std::endl;
    std::cout << argv[0] << " --stop --name Test --path \"C:/Temp\"" << std::endl;
    return 1;
  }

// services:
//
// start
// stop
//
// modes: 
//
// - timecode
// - delay
//
// also needs to send path and filename
//
// plus some description
// plus some notes
//
// finally port
//

  ros::init(argc, argv, "vicon_remote_commander");

	ros::NodeHandle nh("~");
	ros::NodeHandle n; //global nodehandle

	int port_port;
	nh.param<int>("port", port_port, 30);

	std::string vicon_ip;
	nh.param<std::string>("vicon_ip", vicon_ip, "192.168.1.102");

	bool use_broadcast;
	nh.param("use_broadcast", use_broadcast, false);

	Port = (unsigned short)port_port;
	ROS_INFO_STREAM("Using port:" << Port);

	auto startRecordingSrv 	= nh.advertiseService("start_recording", startRecording);
	auto stopRecordingSrv 	= nh.advertiseService("stop_recording", 	stopRecording);
	auto setNamePathSrv 		= nh.advertiseService("set_name_and_path", 	setNamePath);
	
	bool mode_timecode =false;
	nh.param("mode_timecode", mode_timecode, false);
	bool mode_delay =false;
	nh.param("mode_delay", mode_timecode, false);

	int delay_ms = 20;
	nh.param("delay_ms", delay_ms, 20);
	CaptureDelayInMilliseconds = (unsigned int)delay_ms;

	int epoch_start = 20;
	nh.param("epoch_start",epoch_start , 20); //to keep the consistency with savernode
	//Timecode = std::to_string(epoch_start);
	ROS_WARN_STREAM("epoch_start, aka Timecode, is not implemented yet!!!");


	ros::Rate r(10);

  // Seed the random number with the time.
  boost::rand48 Random( 100000 );
  Random.seed( static_cast< boost::rand48::result_type >( time(0) ) );

  // The packet ID is selected randomly.
  // Packets with duplicate ID's are ignored by Nexus.
  const unsigned int CaptureID = Random();

  //.Set up ASIO for UDP broadcast.
  boost::asio::io_service Service;

  // Create and open socket.
  boost::asio::ip::udp::socket Socket( Service );

  boost::system::error_code Error = Socket.open( boost::asio::ip::udp::v4(), Error );
  
  if( !Error )
  {
    // Successfully opened the socket.
    // Set to broadcast.
    Socket.set_option( boost::asio::ip::udp::socket::reuse_address(true) );
    Socket.set_option( boost::asio::socket_base::broadcast(true) );

    boost::asio::ip::udp::endpoint remoteendpoint( boost::asio::ip::address::from_string(vicon_ip), Port );
    boost::asio::ip::udp::endpoint SenderEndpoint( boost::asio::ip::address_v4::broadcast(), Port );
    

	if(use_broadcast)
		remoteendpoint = SenderEndpoint;

    // Build up XML contents of the broadcast message.


while(true){

    std::ostringstream Stream;


	CaptureName = filename;
	CaptureDatabasePath = dirname;
	CaptureNotes = notes;
	CaptureDescription = description;



    // Start capture.
    if( start )
    {
	    start= false;
      Stream << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>" << std::endl;
      Stream << "<CaptureStart>" << std::endl;
      Stream << "  <Name VALUE=\"" << CaptureName << "\"/>" << std::endl;
      if( !CaptureNotes.empty() )
        Stream << "  <Notes VALUE=\"" << CaptureNotes << "\"/>" << std::endl;
      if( !CaptureDescription.empty() )
        Stream << "  <Description VALUE=\"" << CaptureDescription << "\"/>" << std::endl;
      if( !Timecode.empty() )
        Stream << "  <TimeCode VALUE=\"" << Timecode << "\"/>" << std::endl;
      Stream << "  <DatabasePath VALUE=\"" << CaptureDatabasePath << "\"/>" << std::endl;
      Stream << "  <Delay VALUE=\"" << CaptureDelayInMilliseconds << "\"/>" << std::endl;
      Stream << "  <PacketID VALUE=\"" << CaptureID << "\"/>" << std::endl;
      Stream << "</CaptureStart>" << std::endl;
    }

    // Stop capture.
    if( stop )
    {
	    stop = false;
	    Stream << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>" << std::endl;
      Stream << "<CaptureStop>" << std::endl;
      Stream << "  <Name VALUE=\"" << CaptureName << "\"/>" << std::endl;
      Stream << "  <DatabasePath VALUE=\"" << CaptureDatabasePath << "\"/>" << std::endl;
      if( !Timecode.empty() )
        Stream << "  <TimeCode VALUE=\"" << Timecode << "\"/>" << std::endl;
      Stream << "  <PacketID VALUE=\"" << CaptureID << "\"/>" << std::endl;
      Stream << "</CaptureStop>" << std::endl;
    }

    std::string String( Stream.str() );

    // Send message.
    // Buffer must contain terminating 0.
    Socket.send_to( boost::asio::buffer( String.c_str(), String.size() + 1 ), SenderEndpoint );

	r.sleep();

}


	boost::system::error_code close_return_value = Socket.close( Error );

	if (close_return_value)
		std::cout << "Something weird happened" << close_return_value.value() << std::endl;
  }

  return Error.value();
}
