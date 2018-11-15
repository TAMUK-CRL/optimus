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
//   Variable control response(VCR) for adjusting AIS controlled velocity effectiveness.
//   lmvUc= nth state left motor velocity from AIS; rmvUc= nth state right motor velocity from AIS.
//   lmvCp= n-1 state left motor velocity from VCR; rmvCp= n-1 state right motor velocity from VCR.
//   mF= multiplying factor from zFunction with distance to closest obstacle as the variable.
//   cr[0]=delta for left motor; cr[1]= delta for right motor;
//

//
// g++ rosVCR.cpp -o vcr_node -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization
//

// Include the ROS C++ APIs and math library
#include<math.h>
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<nav_msgs/Odometry.h>

std_msgs::Float32MultiArray lmvC,rmvC;
double robotSpeed,mF,iT;
long int a=1000,b=2500,bM=0;

// Subscriber callback
void vesBValCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	// Get max sensing range from 'ves_node'
	//float r.layout.dim[0].size=182;
	//float r.layout.dim[0].stride=1;
	bM=msg->data.back();//Use back() to pull last element
	zFunc();
}

void zFunc()
{
	//double iT;
	// Z-function equation, refer Mathworks.com help for zmf under fuzzy for more details
	if(bM<=a)
	{
		mF=1;
	}
	else if(a<=bM && bM<=(a+b)/2)
	{
		iT=(bM-a)/(b-a);
		mF=1-2*pow(iT,2.0);
	}
	else if((a+b)/2<=bM && bM<=b)
	{
		iT=(bM-b)/(b-a);
		mF=2*pow(iT,2.0);
	}
	else
	{
		mF=0;
	}
	//return mF;
}

// Standard C++ entry point
int main(int argc, char** argv)
{
	// Announce this program to the ROS master
	ros::init(argc, argv, "vcr_node");

	// Start the node resource managers (communication, time, etc)
	ros::NodeHandle vcrNH;// Create node handle

	ros::Rate loop_rate(0.5);

	ros::Publisher vcr_lmvC_pub=vcrNH.advertise<std_msgs::Float32MultiArray>("vcrLMVC",100);
	ros::Publisher vcr_rmvC_pub=vcrNH.advertise<std_msgs::Float32MultiArray>("vcrRMVC",100);
	ros::Subscriber vesBVal=vcrNH.subscribe("/ves_node/vesEnv",100,vesBValCB);

	lmvC.layout.dim.push_back(std_msgs::MultiArrayDimension());
	lmvC.layout.dim[0].label="vcrLeftWheelControl";
	lmvC.layout.dim[0].size=2;
	lmvC.layout.dim[0].stride=1;
	lmvC.layout.data_offset=0;

	rmvC.layout.dim.push_back(std_msgs::MultiArrayDimension());
	rmvC.layout.dim[0].label="vcrRightWheelControl";
	rmvC.layout.dim[0].size=2;
	rmvC.layout.dim[0].stride=1;
	rmvC.layout.data_offset=0;

	int count=0;
	while(ros::ok())
	{
		if(count==0)
		{
			// Clear control vectors
			lmvC.data.clear();
			rmvC.data.clear();

			// Push value of zero in them
			lmvC.data.push_back(0.0);// Check for a better way to do this
			lmvC.data.push_back(0.0);

			rmvC.data.push_back(0.0);
			rmvC.data.push_back(0.0);

		}

		if(bM==0)
		{
			// Calculate controlled velocities
			//	cr[0]=lmvCp+(mF*(lmvUc-lmvCp));// vcr(ArRobot *thisRobot,double cr[2],double lmvUc,double rmvUc,double mF, double lmvCp,double rmvCp)
			//  cr[1]=rmvCp+(mF*(rmvUc-rmvCp));
			lmvC.data.push_back();
			rmvC.data.push_back();
		}


		// Broadcast log message
		//ROS_INFO_STREAM("");
		vcr_lmvC_pub.publish(lmvC);
		vcr_rmvC_pub.publish(rmvC);

		// Process ROS callbacks until receiving a SIGINT (ctrl-c)
		ros::spinOnce();// Check with ros::spin() if spinOnce() doesnt work

		// sleep
		loop_rate.sleep();
		++count;
	}

	// Exit tranquilly
	return 0;
}
