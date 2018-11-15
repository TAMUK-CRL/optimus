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
//
//
//
//
//
//


// Add headers


int getSonar(ArRobot *thisRobot,int r)
{
	ArSensorReading* sonarReading;
	sonarReading = thisRobot->getSonarReading(r);
	return(sonarReading->getRange());
}

int *getLaserObs(ArRobot *thisRobot,int lsrObs[8])	// Corrected version
{
	double *R=aes(thisRobot,range,2300);			// Get Threashold array from AES(4500)
	for(int il=0;il<8;il++)
	{
		if(il==0)
		{
			for(int j=180;j>=157;j--)
			{
				if(scan[j]<=int(R[j]))
					{
						lsrObs[il]=1;
						break;
				}
				else
					{
						lsrObs[il]=0;
				}
			}
		}
		if(il=1)
		{
			for(int j=157;j>=135;j--)
			{
				if(scan[j]<=int(R[j]))
					{
						lsrObs[il]=1;
						break;
				}
				else
					{
						lsrObs[il]=0;
				}
			}
		}
		if(il=2)
		{
			for(int j=135;j>=112;j--)
			{
				if(scan[j]<=int(R[j]))
					{
						lsrObs[il]=1;
						break;
				}
				else
					{
						lsrObs[il]=0;
				}
			}
		}
		if(il=3)
		{
			for(int j=112;j>=90;j--)
			{
				if(scan[j]<=int(R[j]))
					{
						lsrObs[il]=1;
						break;
				}
				else
					{
						lsrObs[il]=0;
				}
			}
		}
		if(il=4)
		{
			for(int j=90;j>=68;j--)
			{
				if(scan[j]<=int(R[j]))
					{
						lsrObs[il]=1;
						break;
				}
				else
					{
						lsrObs[il]=0;
				}
			}

		}
		if(il=5)
		{
			for(int j=68;j>=45;j--)
			{
				if(scan[j]<=int(R[j]))
					{
						lsrObs[il]=1;
						break;
				}
				else
					{
						lsrObs[il]=0;
				}
			}
		}
		if(il=6)
		{
			for(int j=45;j>=23;j--)
			{
				if(scan[j]<=int(R[j]))
					{
						lsrObs[il]=1;
						break;
				}
				else
					{
						lsrObs[il]=0;
				}
			}
		}
		if(il=7)
		{
			for(int j=23;j>=0;j--)
			{
				if(scan[j]<=int(R[j]))
					{
						lsrObs[il]=1;
						break;
				}
				else
					{
						lsrObs[il]=0;
				}
			}
		}
	}
	return lsrObs;
}
int read_back(ArRobot *thisRobot,int sensor_number)			// This code defines the sensor groups and compares with AES threshold values
															// for reverse operation
{
	double *R=aes(thisRobot,range,2300);					// Get AES thresholds
	switch(sensor_number)									// Robot left - right with respect to the back side
	{
	case 0:
		if(getSonar(thisRobot,8)<((R[180]+R[157]+R[168])/3))// Defines onboard sensor 8 as sensor 0, and defines threshold values.
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 1:
		if(getSonar(thisRobot,9)<((R[157]+R[135]+R[146])/3))// Defines onboard sensor 9 as sensor 1, and defines threshold values.
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 2:
		if(getSonar(thisRobot,10)<((R[135]+R[112]+R[123])/3))// Defines onboard sensor 10 as sensor 2, and defines threshold values.
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 3:
		if(getSonar(thisRobot,11)<((R[112]+R[90]+R[101])/3))// Defines onboard sensor 11 as sensor 3, and defines threshold values.
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 4:
		if(getSonar(thisRobot,12)<((R[90]+R[67]+R[78])/3))// Defines onboard sensors 12 as sensor 4, and defines threshold values.
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 5:
		if(getSonar(thisRobot,13)<((R[67]+R[45]+R[56])/3))// Defines onboard sensors 13 as sensor 5, and defines threshold values.
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 6:
		if(getSonar(thisRobot,14)<((R[45]+R[22]+R[33])/3))// Defines onboard sensors 14 as sensor 6, and defines threshold values.
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 7:
		if(getSonar(thisRobot,15)<((R[22]+R[0]+R[11])/3))// Defines onboard sensors 15 as sensor 7, and defines threshold values.
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	default:
		return(0);
	}
}
int read(ArRobot *thisRobot,int sensor_number)// This code defines the sensor groups and compares with AES threshold values
{
	int *lsr=getLaserObs(thisRobot,laserObs);
	switch(sensor_number)// Robot left - right
	{
	case 0:
		if(lsr[0]==1)
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 1:
		if(getSonar(thisRobot,1)<(target*1) || lsr[1]==1)
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 2:
		if(getSonar(thisRobot,2)<(target*1) || lsr[2]==1)
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 3:
		if(getSonar(thisRobot,3)<(target*1.5) || lsr[3]==1)
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 4:
		if(getSonar(thisRobot,4)<(target*1.5) || lsr[4]==1)
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 5:
		if(getSonar(thisRobot,5)<(target*1) || lsr[5]==1)
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 6:
		if(getSonar(thisRobot,6)<(target*1)|| lsr[6]==1)
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	case 7:
		if(lsr[7]==1)
		{
			return(1);
		}
		else
		{
			return(0);
		}
		break;
	default:
		return(0);
	}
}


int main()
{
  // init Node
  // add publisher
  // add subscriber
  // get zone array
}
