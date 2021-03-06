
#include <string>
#include <vector>
#include <iostream>
#include <cstdio>

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
#include "serial_amaury/GPS_raw.h"

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
	
  
    cerr << "Usage: test_serial<baudrate> [test string]" ;
cerr << " {-e|<serial port address>} pour chaque port " << endl;
}

int run(int argc, char **argv)
{
  if(argc < 2) {
	  print_usage();
    return 0;
  }

  //DEBUT DE LA PARTIE AJOUTE

  // Argument 1 is the baudrate
  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[1], "%lu", &baud);
#else
  sscanf(argv[1], "%lu", &baud);
#endif

  // Argument 2 to 5 are the serial ports or enumerate flag
  int nombrePortsGPS = argc-2;
  
  for (int i=2 ; i<argc ; i++){
  	string port(argv[i]);

  	if( port == "-e" ) {
		  enumerate_ports();
		  return 0;
  	}
  	else if( argc < 3 ) {
		  print_usage();
		  return 1;
  	}
  }
  
  
  vector<serial::Serial*> maListe;
  
  maListe.reserve (nombrePortsGPS);
  
  for (int i=0; i<nombrePortsGPS; i++){
  	string port(argv[i+2]);
  	maListe.push_back ( new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000)));

  	cout << "Is the serial port open?";
  	if(maListe[i]->isOpen())
  	  cout << " Yes." << endl;
  	else
    	cout << " No." << endl;
  }
  
  
  
	ros::init(argc, argv, "talkerGPS");

  ros::NodeHandle n;
  
  

  ros::Publisher chatter_pub = n.advertise<serial_amaury::GPS_raw>("GPS_raw_multiple", 1000);

  ros::Rate loop_rate(10);

  
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
   
		
		serial_amaury::GPS_raw msg;
		
		vector<string> result;
		result.reserve (4);
		
		for(int i=0;i<4;i++)result.push_back("");
		
		for(int i =0; i<nombrePortsGPS; i++){
			result[i]=maListe[i]->readline();
		}
		
		
		msg.gps1 = result[0];
		msg.gps2 = result[1];
		msg.gps3 = result[2];
		msg.gps4 = result[3];

		//my_serial.readline();


    ROS_INFO("GPS 1 : %s", msg.gps1.c_str());
		ROS_INFO("GPS 2 : %s", msg.gps2.c_str());
		ROS_INFO("GPS 3 : %s", msg.gps3.c_str());
		ROS_INFO("GPS 4 : %s", msg.gps4.c_str());
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
	
  }

  //FIN DE LA PARTIE AJOUTE

  /* CECI EST LA PARTIE TEST RECUPERE DE SERIAL_EXAMPLE


  // Test the timeout, there should be 1 second between prints
  cout << "Timeout == 1000ms, asking for 1 more byte than written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length()+1);

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }

  // Test the timeout at 250ms
  my_serial.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
  count = 0;
  cout << "Timeout == 250ms, asking for 1 more byte than written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length()+1);

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }

  // Test the timeout at 250ms, but asking exactly for what was written
  count = 0;
  cout << "Timeout == 250ms, asking for exactly what was written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length());

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }

  // Test the timeout at 250ms, but asking for 1 less than what was written
  count = 0;
  cout << "Timeout == 250ms, asking for 1 less than was written." << endl;
  while (count < 10) {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length()-1);

    cout << "Iteration: " << count << ", Bytes written: ";
    cout << bytes_wrote << ", Bytes read: ";
    cout << result.length() << ", String read: " << result << endl;

    count += 1;
  }

  FIN DE LA PARTIE TEST DE SERIAL_EXAMPLE*/

  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}
