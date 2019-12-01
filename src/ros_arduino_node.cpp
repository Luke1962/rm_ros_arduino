/*
Nodo che si interfaccia via seriale ad arduino
derivato da ros_bridge_arduino
Lato ROS
subscribe: cmd_vel
publish: odom, pir,pir1,bumpers

Interfaccia vs Arduino
TX:
pid parameters
pid demand
servo raspicam


RX
encoders count
bumbers

*/

// compilare con:
// catkin_make -DCATKIN_WHITELIST_PACKAGES="rm_ros_arduino"

// launch debug con gdb
//    <node name="ros_arduino" pkg="rm_ros_arduino" type="ros_arduino_node" launch-prefix="gdb -ex run --args"  output="screen">
  
#include <ros/ros.h>

#include <sys/time.h>

#include <geometry_msgs/Twist.h> // comandi
#include <iostream>				 // std::cout
#include <mutex>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <string> // std::string, std::to_string
using namespace std; 

#include <thread>
//--------------------
#include <cstdio>
#include <iostream>
#include <math.h>

//https://github.com/wjwwood/serial/blob/master/include/serial/serial.h
#include <serial/serial.h>
#include <unistd.h>
serial::Serial ser;
//---------------------
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

/*
        git clone https://github.com/jbeder/yaml-cpp/blob/master/install.txt
        cd yaml-cpp
        mkdrir build
        cd build
        cmake ..
        make
        sudo make install

	#include "yaml-cpp/yaml.h"
 */


// COMANDI ------------------
#define CMD_ANALOG_READ "a"
#define CMD_GET_BAUDRATE "b"
#define CMD_PIN_MODE "c"
#define CMD_DIGITAL_READ "d"
#define CMD_READ_ENCODERS "e"
#define CMD_HELP "h"
#define CMD_MOTOR_SPEEDS "m"
#define CMD_PING "p"
#define CMD_RESET_ENCODERS "r"
#define CMD_SERVO_WRITE "s"
#define CMD_SERVO_READ "t"
#define CMD_UPDATE_PID "u"
#define CMD_DIGITAL_WRITE "w"
#define CMD_ANALOG_WRITE "x"
#define CMD_TESTMOTORS "T"
#define CMD_PIDRATE "R"
#define CMD_MOTOR_RPS "M"
#define CMD_READ_PID "U"
#define CMD_SAFE_CMD "S"
#define LEFT 0
#define RIGHT 1
#define CMD_SEPARATOR " "
#define PORT_DIR_IN 0
#define PORT_DIR_OUT 1

//----------------------------
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
/// M A C R O
///  
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
	#define dbgf(...) ROS_DEBUG(__VA_ARGS__)
	# define ROS_NOW  ros::Time::now(); /*Arduino IDE: millis();*/



template <typename T>
int sgn(T val) { return (T(0) < val) - (val < T(0)); }
////////////////////////////////////////////////////////////////////////////////////
// mappa 'value' espresso nel range [istart,istop] nel dominio [ostart,ostop]
float remap(float value, float istart, float istop, float ostart, float ostop)
{
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}



//######################################################
// Inizializza la porta seriale usando i parametri e la apre
//######################################################
bool initSerial(string serial_port, string serial_speed){

	// SERIAL  port, baudrate, timeout in milliseconds
	try
	{
		ROS_INFO_STREAM("Setting Serial Port...");
		ser.setPort(serial_port);
		ser.setBaudrate(std::stoi(serial_speed));
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return false;
	}

	if (ser.isOpen())
	{
		ROS_INFO_STREAM("Serial Port initialized");
		return true;
	}
	else
	{
		return false;
	}	
}




//######################################################
//######################################################
// COMANDI VERSO ARDUINO SU SERIALE CON ATTESA ACK
//######################################################
//######################################################
//#define MAX_ARDUINO_PIN 50
#define DEFAULT_RX_TIMEOUT_SEC 0.010f
#define MAX_RX_RETRIES 3
#define MAX_TX_RETRIES 2

std::string rx_string(float timeout_sec = DEFAULT_RX_TIMEOUT_SEC){
	std::string result;

	result = ser.readline();
	dbgf("\nRX<--:%s", result.c_str());
	return result;
	/*			
		//ser.flushInput();
		ros::Time timeout =ros::Time::now() + ros::Duration(timeout_sec);
		bool blMsgReceived = false;
		while ((ros::Time::now() < timeout) && (!blMsgReceived))
		{
			if (ser.available())
			{
				// ROS_INFO_STREAM("Reading from serial port");

				result = ser.readline();
				dbgf("\nRX<--:%s", result.c_str());
				blMsgReceived = true;

				// ROS_INFO_STREAM("Read: " << result);
			}
		}
		return result;	

	*/
}


bool wait_ack(float timeout_sec = DEFAULT_RX_TIMEOUT_SEC)
{
	std::string ack = rx_string(timeout_sec);
	//if (ack.compare("OK\r") == 1)	{
		if ((ack[0]=='O') && (ack[1]=='K')){
		dbgf("\nACK");return true; 
		
		}
	else{
		return false;	
		}
}
inline bool tx(std::string cmd)
{
	ser.flushOutput();
	cmd += "\r";	
	ser.write(cmd.c_str());
	dbgf("\n-->TX: %s",cmd.c_str() );
}
bool tx_ack(std::string cmd)
{
	bool blAck=false;
	int retries = 0;
	while (!blAck && (retries < MAX_TX_RETRIES))
	{
		
		tx(cmd);	
		
		blAck = wait_ack();
		if (!blAck)
		{
			retries++;
			
		}
			
		//sleep(0.002f);
	}	

}
//######################################################
typedef struct
{
	std::string name;
	std::string specs; // pin , type, rate , direction
	int pin;
	std::string type;
	int direction;
	int rate;
	bool used;
	int value;

} Sensor_t;

boost::mutex mtx_serial;

//######################################################
//######################################################
// CLASSE Ros_arduino
//######################################################
//######################################################
class Ros_arduino
{
public:
	Ros_arduino(ros::NodeHandle &nh);
	//~Ros_arduino();

private:
	// dati della porta seriale
	std::string serial_port;
	std::string serial_speed;

	//	std::mutex serial_mutex;
	// nodo
	float par_rate; // freq. nodo

	// Publisher
	ros::Publisher pub_pir;
	ros::Publisher pub_pir1;
	ros::Publisher pub_chatter;
	//ros::Publisher pub_sensors_in[MAX_ARDUINO_PIN];
	ros::Publisher pub_EncL;
	ros::Publisher pub_EncR;	

	ros::Publisher pub_encoders;	




	// Subscribes
	ros::Subscriber sub_cmdvel;
	ros::Subscriber sub_faretto;
	ros::Subscriber sub_servo_raspicam;
	//ros::Subscriber sub_sensors_out[MAX_ARDUINO_PIN];


	// Parametri
	float rate_sensors;
	float wheel_diameter;
	float wheel_track;
	float encoder_resolution;
	float gear_reduction;
	float Kp;
	float Kd;
	float Ki;
	float Ko;
	float accel_limit;
	bool motors_reversed;	// true se i motori sono invertiti
	int encL;
	int encR;
	//std::string config_file; // nome file yaml nel folder config
	//Sensor_t sensors[MAX_ARDUINO_PIN];

	geometry_msgs::Vector3Stamped msg_encoders;

	void getAndPublishPir();
	void getAndPublishPir1();
	void getAnPublishEncoders();
	void getAndPublishDigitalInput();

	// Callback
	void cbk_cmdvel(const geometry_msgs::Twist cmdvel);

	void cbk_faretto(const std_msgs::Bool msg);
	void cbk_servo_raspicam(const std_msgs::Int16 msg);
	/*  tentativi falliti -*********************************************
		//	void cbk_sensor_out(const ros::MessageEvent<std_msgs::Bool const> &event,const std::string &topic);
		//	void cbk_sensor_out(const ros::MessageEvent<std_msgs::String const>& event);
		void cbk(const ros::MessageEvent<std_msgs::String const>& event);
			void cbk1(const std_msgs::String::ConstPtr &msg,
					const std::string &topic);
	******************************************************************** */

};



/*  tentativi falliti -**************************************
	void Ros_arduino::cbk1(const std_msgs::String::ConstPtr &msg,
						const std::string &topic) {
	; // Do stuff
	}
	void Ros_arduino::cbk(const ros::MessageEvent<std_msgs::String const> &event) {
		const std::string &publisher_name = event.getPublisherName();
		const ros::M_string &header = event.getConnectionHeader();
		ros::Time receipt_time = event.getReceiptTime();

		const std_msgs::StringConstPtr &msg = event.getMessage();
	} 


	//nh.subscribe(sensors[i].name, 1,boost::bind( &Ros_arduino::cbk_sensor_out,
	this, _1,sensors[i].name));
	//void Ros_arduino::cbk_sensor_out(std_msgs::Bool msg, const std::string &topic)
	{

	//void Ros_arduino::cbk_sensor_out(const ros::MessageEvent<std_msgs::Bool const>
	&event,
	//								 const
	std::string &topic)



	void Ros_arduino::cbk_sensor_out(const ros::MessageEvent<std_msgs::String
	const>& event)
	{

			ros::M_string connection_header = event.getConnectionHeader();
			std::string strTopic = connection_header["topic"];

			std_msgs::Bool  msgPtr;
			//msgPtr = event.getMessage();
			dbgf("received topic: %s", strTopic);

			//const std::string &publisher_name = event.getPublisherName();
			//ROS_INFO("publisher: %s\ttopic: %s", publisher_name.c_str(),topic.c_str());

			// converto da topic a pin-------------------------
			for (size_t i = 0; i < MAX_ARDUINO_PIN; i++)
			{
					if (sensors[i].name == strTopic)
					{
							std::string cmd =CMD_DIGITAL_WRITE + CMD_SEPARATOR + sensors[i].pin + msgPtr.data; tx_ack(cmd);
							dbgf(cmd.c_str());
					}
			}


	}


*********************************************************************** */
//--------------------------------------------------------
// faretto C A L L B A C K 
//--------------------------------------------------------
//#define PIN_FARETTO "12"
#define PIN_FARETTO "30"
void Ros_arduino::cbk_faretto(const std_msgs::Bool msg){
	string strValue;
	if (msg.data==true)
	{
		strValue = "1";
	}else
	{
		strValue = "0";
	}
	
	
	const string cmd=  (string)CMD_DIGITAL_WRITE 
						+(string)CMD_SEPARATOR  
						+(string)PIN_FARETTO
						+(string)CMD_SEPARATOR
						+(string)strValue;
	dbgf("\nCBK Faretto");
	mtx_serial.lock();
	tx_ack(cmd);
	mtx_serial.unlock();

}


//--------------------------------------------------------
// cmd_vel C A L L B A C K 
//--------------------------------------------------------
void Ros_arduino::cbk_cmdvel(geometry_msgs::Twist cmdvel) {

	float x = cmdvel.linear.x;
	float th = cmdvel.angular.z;
	float left;
	float right;

	if (x == 0.0f) {
	// Turn in place
	right = th * wheel_track / 2.0;
	left = -right;
	} else if (th == 0.0f) {
	// Pure forward/backward motion
	left = x;
	right = x;
	} else {
	// Translation + Rotation
	left = x - th * wheel_track / 2.0;
	right = x + th * wheel_track / 2.0;
	}

	// ------------------------
	// trasformo le velocità delle ruote in velocità asse motore--
	float left_revs_per_second = gear_reduction * left / (wheel_diameter * M_PI);
	float right_revs_per_second =
		gear_reduction * right / (wheel_diameter * M_PI);

	std::string cmd= (string)CMD_MOTOR_SPEEDS 
		+(string)CMD_SEPARATOR
		+ std::to_string(left_revs_per_second) 
		+(string)CMD_SEPARATOR 
		+ std::to_string(right_revs_per_second);

	// ROS_INFO_THROTTLE(1.0, cmd);
	dbgf("\nCBK cmd_vel");
	mtx_serial.lock();
	tx_ack(cmd);
	mtx_serial.unlock();
	// dbgf(cmd.c_str());
}


//--------------------------------------------------------
// lettura e pubblicazione Encoders
//--------------------------------------------------------
void Ros_arduino::getAnPublishEncoders(){

	std::string cmd =(string)CMD_READ_ENCODERS;

	mtx_serial.lock();
	tx(cmd);
	// attendo la ricezione dei valori dei due encoders
	std::string  strCallback = rx_string();
	mtx_serial.unlock();
	//dbgf("\n\tcbk from arduino: %s", strCallback.c_str());

	//converto la stringa nei due valori
	std::string::size_type sz;

	msg_encoders.header.stamp = ros::Time::now();
	try
	{
		msg_encoders.vector.x =(float)  stoi(strCallback , &sz);
		
		msg_encoders.vector.y =  (float)stoi(strCallback.substr(sz), &sz );
		
	}
	catch(const std::exception& e)
	{
		std::cerr<< "Encoders Exception ["<<strCallback<<"]" << e.what() << '\n';
	}
	

	pub_encoders.publish(msg_encoders);

	//dbgf("\nEnc1 : %f, Enc2 %f",msg_encoders.vector.x ,msg_encoders.vector.y);


}


//--------------------------------------------------------
// P I R
//--------------------------------------------------------
#define PIN_PIR "33"
void Ros_arduino::getAndPublishPir(){
	std::string cmd =(string)CMD_DIGITAL_READ +(string)CMD_SEPARATOR +(string)PIN_PIR;
	mtx_serial.lock();
	tx(cmd);
	// attendo la ricezione dei valori dei due encoders
	std::string  strCallback = rx_string();
	mtx_serial.unlock();

	std_msgs::Bool msg;

	try
	{
		msg.data = (bool)stoi(strCallback );	 
	}
	catch(const std::exception& e)
	{
		std::cerr << "***Pir EXCEPTION*** ["<< strCallback  <<"]"<< e.what() << '\n';
	}
 
	pub_pir.publish(msg);

}


//--------------------------------------------------------
// P I R 1
//--------------------------------------------------------
#define PIN_PIR1 "34"
void Ros_arduino::getAndPublishPir1(){
	std::string cmd =(string)CMD_DIGITAL_READ +(string)CMD_SEPARATOR +(string)PIN_PIR1;
	mtx_serial.lock();
	tx(cmd);
	// attendo la ricezione dei valori dei due encoders
	std::string  strCallback = rx_string();
	mtx_serial.unlock();

 
	std_msgs::Bool msg;
	try
	{
		msg.data = (bool)stoi(strCallback );
	}
	catch(const std::exception& e)
	{
		std::cerr << "***Pir 1 EXCEPTION*** ["<< strCallback  <<"]"<< e.what() << '\n';
		 
	}
		
	pub_pir1.publish(msg);

}
//--------------------------------------------------------
// SERVO
//--------------------------------------------------------

#define PIN_SERVO_RASPICAM "9"
void Ros_arduino::cbk_servo_raspicam(std_msgs::Int16 msg){

	string strValue = std::to_string( msg.data);

	const string cmd=  (string)CMD_SERVO_WRITE
						+(string)CMD_SEPARATOR  
						+(string)PIN_SERVO_RASPICAM
						+(string)CMD_SEPARATOR
						+(string)strValue;
	dbgf("\nCBK Servo Raspicam %s",strValue.c_str() );
	mtx_serial.lock();
	tx_ack(cmd);
	mtx_serial.unlock();

}


//--------------------------------------------------------
// lettura e pubblicazione Digital Inputs
//--------------------------------------------------------
void Ros_arduino::getAndPublishDigitalInput(){
	getAndPublishPir();
	getAndPublishPir1();
}


////////////////////////////////////////////////////////////////////////////////////
// Costruttore
Ros_arduino::Ros_arduino(ros::NodeHandle &nh)
{

	// inizializzo le variabili
	//for (size_t i = 0; i < MAX_ARDUINO_PIN; i++){sensors[i].used = false;}

	//-----------------------------------------------------------------
	// Leggo i parametri (launch file)
	//-----------------------------------------------------------------
	nh.param<float>("rate", par_rate, 10);
	nh.param<std::string>("serial_port", serial_port, "/dev/tty/ACM0");
	nh.param<std::string>("serial_speed", serial_speed, "115200");
	nh.param<float>("wheel_diameter", wheel_diameter, 1);
	nh.param<float>("wheel_track", wheel_track, 1);
	nh.param<float>("encoder_resolution", encoder_resolution, 1);
	nh.param<float>("gear_reduction", gear_reduction, 1);


	nh.param<float>("rate_sensors", rate_sensors, 10);
	ros::Duration  interval_sensors = ros::Duration( 1/rate_sensors);
	ros::Time nex_time_sensors  = ros::Time::now() ;
	/*

			//-----------------------------------------------------------------
			// gestione file yaml
			//-----------------------------------------------------------------
			//nh.param<std::string>("config_file", config_file, "ros_arduino.yaml");
			YAML::Node yaml = YAML::LoadFile(config_file);
			XmlRpc::XmlRpcValue paramList;

			int pin;

			if (nh.getParam("sensors", paramList)) // se esiste la sezione sensors
			{
				std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
				for (i = paramList.begin(); i != paramList.end(); i++)
				{
					// sensor_specs.assign(i->second["pin"]);
					pin = i->second["pin"];

					sensors[pin].name = i->first;
					sensors[pin].type = static_cast<std::string>(i->second["type"]);
					sensors[pin].rate = i->second["rate"];
					if (static_cast<std::string>(i->second["direction"]) == "input")
					{
						sensors[pin].direction = PORT_DIR_IN;
					}
					else
					{
						sensors[pin].direction = PORT_DIR_OUT;
					}

					sensors[pin].used = true;

					ROS_INFO("\nSensor: %s : pin %d,  type:%s ,rate: %d , direction %d",
							sensors[pin].name.c_str(), pin, sensors[pin].type.c_str(),
							sensors[pin].rate, sensors[pin].direction);
				}
				dbgf("\n");
			}
			else
			{
				ROS_INFO("NO SECTION sensors");
			}

	*/

	//-----------------------------------------------------------------
	// subscribes
	//-----------------------------------------------------------------

    sub_cmdvel = nh.subscribe("/cmd_vel", 1, &Ros_arduino::cbk_cmdvel, this,
							  ros::TransportHints().tcpNoDelay());


	sub_faretto = nh.subscribe("/faretto", 1, &Ros_arduino::cbk_faretto, this,
							  ros::TransportHints().tcpNoDelay());

	sub_servo_raspicam = nh.subscribe("/servo_raspicam", 1, &Ros_arduino::cbk_servo_raspicam, this,
							  ros::TransportHints().tcpNoDelay());

	//-----------------------------------------------------------------
	// advertise publishes
	//-----------------------------------------------------------------
	// publish di tutte le porte in input
	pub_pir = nh.advertise<std_msgs::Bool>("/pir", 1);
	pub_pir1 = nh.advertise<std_msgs::Bool>("/pir1", 1);
	//pub_EncL = nh.advertise<std_msgs::Int32>("/encL",1);
	//pub_EncR = nh.advertise<std_msgs::Int32>("/encR",1);
	pub_encoders = nh.advertise<geometry_msgs::Vector3Stamped>("/encoders",1);
	

	/* prove *******************************************************************

		for (size_t i = 0; i < MAX_ARDUINO_PIN; i++)
		{
			if ((sensors[i].used) && (sensors[i].type == "Digital"))
			{
				if (sensors[i].direction == PORT_DIR_IN)
				{ // INPUT
					pub_sensors_in[i] = nh.advertise<std_msgs::Bool>(sensors[i].name, 1);
				}
				else
				{ // output
							const char *tpc = "bumper1";
							sub_sensors_out[i] =
								//   nh.subscribe(sensors[i].name, 1,&Ros_arduino::cbk_sensor_out, this, ros::TransportHints().tcpNoDelay()); 
								// nh.subscribe<std_msgs::String>(sensors[i].name, 1, boost::bind(&Ros_arduino::cbk ,_1, sensors[i].name));

								nh.subscribe<std_msgs::String>(
									sensors[i].name, 1,
									boost::bind(&Ros_arduino::cbk1, this,_1, &sensors[i].name)
								);
				}
				// imposto la direzione della porta su arduino
				std::string cmd =
					CMD_PIN_MODE + std::string(CMD_SEPARATOR) +
					std::to_string(sensors[i].pin) + CMD_SEPARATOR +
					std::to_string(sensors[i].direction);
				tx_ack(cmd);
			}
		}
	******************************************************************* */



	//-----------------------------------------------------------------
	if (initSerial(serial_port,serial_speed))
	{

		ros::Rate rate(par_rate);

		////////////////////////////////////////////////////////////////////////////////////
		// main loop
		////////////////////////////////////////////////////////////////////////////////////
		while (ros::ok())
		{
			getAnPublishEncoders();
/* 
			if (ros::Time::now() > nex_time_sensors )
			{
				getAndPublishDigitalInput();

				nex_time_sensors  = ros::Time::now() + interval_sensors;

			}
 */
			//ROS_INFO_THROTTLE(5, "running");

			ros::spinOnce(); // gestisce i callback
			rate.sleep();
		}

	}




}

//----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_arduino_node");

	ros::NodeHandle nh("~");

	Ros_arduino node = Ros_arduino(nh);

	return 0;
}