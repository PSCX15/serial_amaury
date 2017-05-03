
#include <string>
#include <iostream>
#include <cstdio>
#include <cmath>
#include "nav_msgs/Odometry.h"
// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"


// ajouté
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial_amaury/mesure.h"

#include <sstream>
// fin ajouté


using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
      Sleep(milliseconds); // 100 ms
#else
      usleep(milliseconds*1000); // 100 ms
#endif
}

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;

		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
     device.hardware_id.c_str() );
	}
}

void print_usage()
{
	// Si on demande le baud rate : cerr << "Usage:  <baudrate> ";
		cerr << "Usage: ";
    cerr << " {-e|<serial port address>}" << endl;
}


float x;
float y;


void positionCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;

}


int run(int argc, char **argv)
{
	
	
	ros::init(argc, argv, "talkerParticule");

  ros::NodeHandle n;

  if(argc < 1) {
	  print_usage();
    return 0;
  }

  // Argument 2 is the serial port or enumerate flag
  string port="";
  if(argc >= 2){
  	port = argv[1];
	}
  if( port == "-e" ) {
	  enumerate_ports();
	  return 0;
  }
  else if( argc < 2 ) {
	  print_usage();
	  return 1;
  }
  
  n.getParam("particulePath", port);
  
  
  // On regle le baud à 4800 //
	
	unsigned long baud = 9600;
	
	// On récupère le numéro de gps
	
	
	/////////////////////////////

	/* Pour mettre le baudrate en argument !
	
	
  // Argument 1 is the baudrate
  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[1], "%lu", &baud);
#else
  sscanf(argv[1], "%lu", &baud);
#endif
	
	
	*/
	
	
  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

	/*
  // Get the Test string
  int count = 0;
  string test_string;
  if (argc == 4) {
    test_string = argv[3];
  } else {
    test_string = "Testing.";
  }
  */
  
  //DEBUT DE LA PARTIE AJOUTE
  


  ros::Publisher chatter_pub = n.advertise<serial_amaury::mesure>("donneesParticules", 1000);
  ros::Subscriber position_topic = n.subscribe("/odometry/filtered_map", 1000, positionCallBack);
  ros::Rate loop_rate(10);
	

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    serial_amaury::mesure msg;
    

    //On récupère l'info!
		my_serial.flush();
		string result = my_serial.readline();
		my_serial.flush();
		
		//Parsing

		string delimiter = " ";

		size_t pos = 0;
		string token;
		
		int count = 0;
		vector<string> resultVec;
		resultVec.reserve(20);
		
		// Le contenu de la string est mise dans un vector
		
		
		while ((pos = result.find(delimiter)) != string::npos) {
   		token = result.substr(0, pos);
   	 	resultVec.push_back(token);
   	 	count++;
    	result.erase(0, pos + delimiter.length());
		}
		
		// Ensuite on trie les infos pour les mettre dans un msg de la classe GPS_refined
		
		if(!resultVec[0].compare("concentration")){
			
			msg.a = std::atof(resultVec[2].c_str());
			msg.x = x;
			msg.y = y;
			
			chatter_pub.publish(msg);
			
		}

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
		

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
	
  }
  


  //FIN DE LA PARTIE AJOUTE

  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (exception &e) {

    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}
