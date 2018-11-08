//
//   Copyright 2018 Pranav Phadke
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//       http://www.apache.org/licenses/LICENSE-2.0
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
//   Variable Environment Sensing(AES) generates threshold values used for comparision with measured distances
//   r[0 to 180]= Threshold values; r[181]= max sensing range at current speed
//   a= radius of minor axis of ellipse (set to 400mm); b= radius of major axis at current speed; bMax=Maximum sensing range (defualt set to 1500mm)
//   m= controls minimum sensing range at zero speed, higher the value less is the sensing range (set to 100); c= linearity of the curve, greater the value higher the linearity (set to 80)
//

//
// g++ rosVES.cpp -o ves_node -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization
//

// Include the ROS C++ APIs and math library
#include<math.h>
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<nav_msgs/Odometry.h>

std_msgs::Float32MultiArray r;
double robotSpeed; //r[182];//float32[]
long int m=100,c=80,a=400,bMax=1500;
long int b;

// Subscriber callback
void robotPoseCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	// Get pose and calculate velocity of robot
	robotSpeed=msg->twist.twist.linear.x;// Get speed from the robot microprocessor
}

// Standard C++ entry point
int main(int argc, char** argv)
{
	// Announce this program to the ROS master as a "node" called "hello_world_node"
	ros::init(argc, argv, "ves_node");

	// Start the node resource managers (communication, time, etc)
	//ros::start();
	ros::NodeHandle vesNH;// Create node handle

	ros::Rate loop_rate(0.5);

	ros::Publisher ves_pub=vesNH.advertise<std_msgs::Float32MultiArray>("vesEnv",100);//&r);
	ros::Subscriber robotPose=vesNH.subscribe("/RosAria/pose",100,robotPoseCB);

	//aesNH.advertise(ves_pub,100);

	//r.layout.dim=(std_msgs::MultiArrayDimension *);
	r.layout.dim.push_back(std_msgs::MultiArrayDimension());
	r.layout.dim[0].label="vesRangeValsPlusMaxRange";
	r.layout.dim[0].size=182;
	r.layout.dim[0].stride=1;
	r.layout.data_offset=0;
	//r.data_length=182;

	int count=0;
	while(ros::ok())
	{
		r.data.clear();
		// Calculate envelope limits
		b=bMax/(1+exp((m-robotSpeed)/c));// Sigmoidal function equation with speed as variable
		for(int i=0;i<181;i++)
		{
			r.data.push_back(abs((a*b)/sqrt(pow((b*cos(i*M_PI/180)),2)+pow((a*sin(i*M_PI/180)),2))));// Use this if next one fails
			//r.data[i]=abs((a*b)/sqrt(pow((b*cos(i*M_PI/180)),2)+pow((a*sin(i*M_PI/180)),2)));// Elliptical threshold function r(theta)=ab/sqrt[(bcos(theta))^2+(asin(theta))^2]
		}
		// Put current max sensing range at the end
		r.data.push_back(b);
		//r.data[181]=b;

		// Broadcast log message
		//ROS_INFO_STREAM("Hello, world!");
		ves_pub.publish(r);

		// Process ROS callbacks until receiving a SIGINT (ctrl-c)
		ros::spinOnce();// Check with ros::spin() if spinOnce() doesnt work

		// Clear aesRange and sleep
		loop_rate.sleep();
		++count;
	}

	// Stop the node's resources
	//ros::shutdown();

	// Exit tranquilly
	return 0;
}
