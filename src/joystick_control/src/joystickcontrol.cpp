#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "SDL/SDL.h"
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <pthread.h>			//Multi threading concerning WIFI scanning


//#define PATH "/home/connor/logs/"
#define DTOR(d) ((d) * M_PI / 180)

using namespace std;

float laserRead[181];
bool wifiScan = true;

double ypos=0, xpos=0, theta=0;

void poseCallback(const nav_msgs::Odometry msg){
//  static tf::TransformBroadcaster br;
  tf::Transform transform;
double roll, pitch, yaw;
  //ROS_INFO("Quaternion: x=%f, y=%f, z=%f, w=%f", msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
 ROS_INFO("ROLL:%f, PITCH:%f, Yaw:%f",roll, pitch, yaw);
  xpos = msg.pose.pose.position.x;
  ypos = msg.pose.pose.position.y;
  theta = yaw;
}


void scanValues(const sensor_msgs::LaserScan laser)
{
	for (unsigned int i=0; i<laser.ranges.size();i++)
	{
		//fprintf(stderr,"range[%d]=[%f]: ", i, laser.ranges[i]);
		//ROS_INFO("range:[%f]: %i", laser.ranges[i], i);
		laserRead[i]=laser.ranges[i];
//		cout<< "laser5:\t " << laserRead[5] << endl;
	}
		//ROS_INFO("MAX RANGE IS %f", laser.range_max);
		//ROS_INFO("MIN RANGE IS %f", laser.range_min);
}



#ifdef WIFI
void* scanWifi(void*)
{
	ofstream ofswifi;
	ofswifi.open("/home/connor/logs/wifi.txt",ios::out);
	int i=0;
	struct timeval tvwifi;
	while(wifiScan) {
		//cout<<"Loc: "<<locCount<<" scan "<<i<<" of "<<numReadings<<endl;
		i++;
		gettimeofday(&tvwifi,NULL);

		FILE* stream  = popen("sudo iwlist wlan0 scan","r");
		stringstream output;
		if(stream) {
			while(!feof(stream) && !ferror(stream)) {
				char buf[128];
				int bytesRead = fread(buf,1,128,stream);
				output.write(buf,bytesRead);
			}
			pclose(stream);
		}
		if(output.str().length()>0)
		{
			cout<<"WiFi scan completed"<<endl;
			ofswifi<<"SCAN#"<<i<<" loc#"<<0<<" Time(seconds): "<<fixed<<tvwifi.tv_sec + tvwifi.tv_usec/1000000.0<<endl;
			ofswifi<<output.str()<<endl;
		}
		else {
			cout<<" -> Repeat"<<endl;
			i--;
		}
	}
	ofswifi.close();
	pthread_exit(0);
}
#endif


SDL_Joystick *joystick;	// joystick

int initjoy() {

	const char *name;
	int i;
	
	SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK);
	
	if(SDL_NumJoysticks()==0) {
		printf("No joystick connected -- aborting.\n");
		exit(-1);
	}
	else if(SDL_NumJoysticks()==1) {
		name = SDL_JoystickName(i);
		printf("Only one joystick connected. Using: %s.\n",name ? name : "Unknown Joystick");
		joystick = SDL_JoystickOpen(0);
	}
	else
	{
		printf("There are %d joysticks attached:\n", SDL_NumJoysticks());
		for ( i=0; i<SDL_NumJoysticks(); ++i ) {
			name = SDL_JoystickName(i);
			printf("  %d: %s\n",i,name ? name : "Unknown Joystick");
		}
		printf("Please enter the joystick number you want to use [0-%d]: ", SDL_NumJoysticks()-1);
		int ret = scanf("%d", &i); 
		printf("\n");
		joystick = SDL_JoystickOpen(i);
	}
	
	if ( joystick == NULL ) {
		printf("Error opening joystick -- aborting\n");
		return 1;
	}
	
	return 0;
}

int main(int argc, char** argv)
{
	struct timeval tv;
	bool joyON = true;
	if(initjoy()==0)
	{
		cout << "Joystick initialized" << endl;
		cout << "num of buttons: " << SDL_JoystickNumButtons(joystick) << endl;
		cout << "num of Hats: " << SDL_JoystickNumHats(joystick) << endl;
		cout << "num of balls: " << SDL_JoystickNumBalls(joystick) << endl;
		cout << "num of Axes: " << SDL_JoystickNumAxes(joystick) << endl;
	}
	else 
	{
		cout<<"Joystick initialization failed!"<<endl;
		exit(-1);
	}


	double newspeed = 0, newturnrate = 0;
	//ramping robot motor speeds
	double u[2];
	double ulow[] = {0.15, 0.1};
	double uhigh[] = {0.8, 0.45}; // Default: {0.4, 0.25}
	u[0]=ulow[0];
	u[1]=ulow[1];

  	//init the ROS node
	ros::init(argc, argv, "joystickControl");
	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber od_sub = nh.subscribe("/RosAria/pose", 1000, poseCallback);
	ros::Rate rate(10);
	geometry_msgs::Twist base_cmd;

	#ifdef ODOMETRY
	ofstream ofsodo, ofsinputs;
	ofsodo.open("/home/connor/logs/dOdometry.txt",ios::out);
	ofsinputs.open("/home/connor/logs/dInput.txt",ios::out);
	#endif

	#ifdef WIFI
	//ofstream ofswifi;
	//ofswifi.open("logs/wifi.txt",ios::out);
	bool wifiCapture = true;
	bool wifiPressed = false;
	int locCount = 1;
	double dtWifi = 1;
	struct timeval tvWifi;

	pthread_t mythread;
	pthread_create(&mythread,NULL,scanWifi,NULL);
	#endif


	#ifdef LASER
	//creates subscriber node
	ros::Subscriber laser_sub = nh.subscribe("/scan", 1000, scanValues);
	ofstream ofs2GM;
	ofs2GM.open("/home/connor/logs/REAL_GM.txt",ios::out);
	ofs2GM << "PARAM robot_frontlaser_offset 0.0 nohost 0" << endl;
	ofs2GM << "PARAM laser_front_laser_resolution 0.5 nohost 0" << endl;
	ofs2GM.precision(12);
	#endif


	while(ros::ok() & joyON)
	{

/*		ofstream ofslaser;
		ofslaser.open(PATH,ios::out); //had to use hard path
		ofslaser << "help 2 \n";
		ofslaser.close(); */

		//set all motor states to NULL
		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
		SDL_JoystickUpdate();

	newturnrate =0; newspeed =0;		
	if ((SDL_JoystickGetAxis(joystick, 3)) != 0 || (SDL_JoystickGetAxis(joystick, 2)) !=0){
//routes to the right joystick X-axis on wireless logitech
	newturnrate = -1*u[0]*(double)(SDL_JoystickGetAxis(joystick, 2))/32768;

//routes to the right joystick Y-axis on wireless logitech
	newspeed = u[1]*(double)(SDL_JoystickGetAxis(joystick, 3))/32768*-1;
}

			if(SDL_JoystickGetButton(joystick,2))
			{	
			ROS_INFO("Low Speed Mode");
			u[0]=ulow[0];
			u[1]=ulow[1];
			}

			if(SDL_JoystickGetButton(joystick,3))
			{	
			ROS_INFO("High Speed Mode");
			u[0]=uhigh[0];
			u[1]=uhigh[1];
			}


			if(SDL_JoystickGetButton(joystick,0))
			{	
				ROS_INFO("Setting Drive state to NULL");
				newturnrate = 0;
				newspeed = 0;
			}

			if ( SDL_JoystickGetButton(joystick,1) ){
				cout << "BUTTON 2 PRESSED, QUIT JOYSTICK INITIALIZED" << endl;
				joyON= false;
				ROS_INFO("Setting Drive state to NULL");
				newturnrate = 0;
				newspeed = 0;
			}

			ROS_INFO("newspeed: %f, \t newturn : %f", newspeed, newturnrate);
			base_cmd.linear.x = newspeed;
			base_cmd.angular.z = newturnrate;
			/*//move forward
		base_cmd.linear.x = 0.25;

		if (laserRead[89] < 2.50)
			base_cmd.angular.z = 0.75; //turn left
		*/



			#ifdef LASER
			// log the laser data
			ofs2GM.precision(12);
			ofs2GM << "ODOMETRY "<< xpos << " " << ypos << " " << theta << " " << newspeed << " " << newturnrate <<" 0 " <<tv.tv_sec + tv.tv_usec/1000000.0 << " nohost " << tv.tv_sec+tv.tv_usec/1000000.0<<endl;
			ofs2GM << "FLASER 181 ";
			ofs2GM.precision(5);
			for (  int j = 0 ; j < (sizeof(laserRead)/sizeof(*laserRead)); j+=2 ) {
				ofs2GM  << laserRead[j] << " " ;
			}
			ofs2GM << " " << xpos << " " << ypos << " " << theta << " " << xpos << " " << ypos << " " << theta << " "  <<   tv.tv_sec + tv.tv_usec/1000000.0 << " nohost " << tv.tv_sec   + tv.tv_usec/1000000.0<<  endl;
			#endif

			#ifdef ODOMETRY
			// log the odometry
			ofsodo << fixed << tv.tv_sec + tv.tv_usec/1000000.0;
			ofsodo << "\t" << xpos  << "\t" << ypos << "\t" << theta;
			ofsodo << "\t" << newspeed << "\t" << newturnrate << endl;
			#endif

			#ifdef ODOMETRY
 			ofsinputs<<tv.tv_sec + tv.tv_usec/1000000.0 <<" "<<newspeed<<" "<<newturnrate<<endl;
			#endif


			// end loop publish/rate set
			cmd_vel_pub.publish(base_cmd);
			ros::spinOnce();
			rate.sleep();

	}

	#ifdef WIFI
	wifiScan = false;
	pthread_join(mythread,NULL);
	#endif

	#ifdef ODOMETRY
	ofsodo.close();
	ofsinputs.close();
	#endif

	#ifdef LASER
	ofs2GM.close();
	#endif

      return 0;
}
