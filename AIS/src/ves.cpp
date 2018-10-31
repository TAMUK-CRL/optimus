//
//   Copyright 2013 Pranav Phadke
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
//   Variable Environment Sensing(VES) generates threshold values used for comparision with measured distances
//   r[0 to 180]= Threshold values; r[181]= max sensing range at current speed
//   a= radius of minor axis of ellipse (set to 400mm); b= radius of major axis at current speed; bMax=Maximum sensing range (defualt set to 1500mm)
//   m= controls minimum sensing range at zero speed, higher the value less is the sensing range (set to 100); c= linearity of the curve, greater the value higher the linearity (set to 80)
//

double *ves(ArRobot *thisRobot, double r[182], int bMax=1500)
{
	double speed=abs(thisRobot->getVel()),b=0;		// Get speed from the robot microprocessor and set
	long int m=100,c=80,a=400;
	b=bMax/(1+exp((m-speed)/c));					// Sigmoidal function equation with speed as variable
	for(int i=0;i<181;i++)
	{
		r[i]=abs((a*b)/sqrt(pow((b*cos(i*PI/180)),2)+pow((a*sin(i*PI/180)),2)));// Elliptical threshold function r(theta)=ab/sqrt[(bcos(theta))^2+(asin(theta))^2]
	}
	r[181]=b;
	return r;										// Return AES threshold array
}
