
#include <string>
#include <iostream>
#include <cstdio>
#include <cmath>

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
#include "serial_amaury/CI_msg.h"

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
	cerr << "Usage: test_serial <baudrate> ";
    cerr << " {-e|<serial port address>}" << endl;
}

int run(int argc, char **argv)
{
  if(argc < 2) {
	  print_usage();
    return 0;
  }

  // Argument 2 is the serial port or enumerate flag
  string port(argv[2]);

  if( port == "-e" ) {
	  enumerate_ports();
	  return 0;
  }
  else if( argc < 3 ) {
	  print_usage();
	  return 1;
  }

  // Argument 1 is the baudrate
  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[1], "%lu", &baud);
#else
  sscanf(argv[1], "%lu", &baud);
#endif

  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  // Get the Test string
  int count = 0;
  string test_string;
  if (argc == 4) {
    test_string = argv[3];
  } else {
    test_string = "Testing.";
  }
  //DEBUT DE LA PARTIE AJOUTE
  
	ros::init(argc, argv, "LecteurCI");

  ros::NodeHandle n;

  ros::Publisher chatter_acc = n.advertise<serial_amaury::CI_msg>("CIAcc", 1000);
	ros::Publisher chatter_gyr = n.advertise<serial_amaury::CI_msg>("CIGyr", 1000);
	ros::Publisher chatter_mag = n.advertise<serial_amaury::CI_msg>("CIMag", 1000);
  

  
  ros::Rate loop_rate(2);

  
  while (ros::ok())
  {

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    serial_amaury::CI_msg msg;

		string result = my_serial.readline();

		//Parsing
		


		string delimiter = ",";

		size_t pos = 0;
		string token;
		
		int count = 0;
		vector<string> resultVec;
		resultVec.reserve(4);
		
		// Le contenu de la string est mise dans un vector
		

		while ((pos = result.find(delimiter)) != string::npos) {
   		token = result.substr(0, pos);
   	 	resultVec.push_back(token);
   	 	count++;
    	result.erase(0, pos + delimiter.length());
		}
		
		
		if (resultVec.size()>=4 && !resultVec[3].empty() && resultVec[3][resultVec[3].size() - 1] == '\n'){
    	resultVec[3].erase(resultVec[3].size() - 1);
    	ROS_INFO("le z est %s", resultVec[3].c_str());
    }
		
		  	  
		
		
		
		if(resultVec.size()!=0){
			  	  
			if(!resultVec[0].compare("INFO") && resultVec.size()>1){
				ROS_INFO("%s",resultVec[1].c_str());

			}
			else if(resultVec.size()>=3){

				msg.type = resultVec[0];
				msg.x = std::atof(resultVec[1].c_str());
				 
				msg.y = std::atof(resultVec[2].c_str());
				msg.z = std::atof(resultVec[3].c_str());
				

				
				if(!(msg.type).compare("A")){
					chatter_acc.publish(msg);
					//ROS_INFO(" acc : x=%f ; y=%f ; z=%f",msg.x,msg.y,msg.z);
					//ROS_INFO(" acc : z=%f",msg.z);
				}
				else if(!(msg.type).compare("C")){
					chatter_mag.publish(msg);
					float norme = sqrt(msg.x*msg.x+msg.y*msg.y+msg.z*msg.z);
					ROS_INFO(" mag : x=%f ; y=%f ; z=%f",msg.x,msg.y,msg.z);
					//ROS_INFO(" mag norme : x=%f ; y=%f ; z=%f",msg.x/norme,msg.y/norme,msg.z/norme);
				}
				else if(!(msg.type).compare("G")){
					chatter_gyr.publish(msg);
					//ROS_INFO(" gyr : x=%f ; y=%f ; z=%f",msg.x,msg.y,msg.z);
				}
				else{
					ROS_INFO("chelou, pas de message envoyé");
				}
			}
			else{
				ROS_INFO("message mal recu");
			}

    	ros::spinOnce();
    	//loop_rate.sleep();
    	
    }
	
  }
	
  //FIN DE LA PARTIE AJOUTE


/*
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
*/
  /*FIN DE LA PARTIE TEST DE SERIAL_EXAMPLE*/

  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}