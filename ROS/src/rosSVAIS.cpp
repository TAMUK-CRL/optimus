///
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
//   Robot Navigation with Artificial Immune System using Speed Variable Obstacle Detection
//   Coded by:- Pranav Phadke
//   Revision:- 5.0.0
//   Date created:- 04/30/2013
//   Date created:- 11/15/2018
//

// #include"Aria.h"
#include<stdio.h>
#include<windows.h>
#include<iostream>
#include<cstdlib>
#include<cmath>
#include<ctime>
#include<string>
#include<fstream>
#include<time.h>
#include<bitset>
#include<math.h>
//#include<Eigen/Dense>
#define PI 3.14159265
using namespace std;
using Eigen::MatrixXd;
using namespace Eigen;
void motorvoltages(void);
long int target=550;
int i;
double range[182],controlResponse[2],scan[181];
double scanX[180],scanY[180],robotX,robotY,robotTh,ZtPredict[5][18],ZtObs[3][18];
int laserObs[8];
ArSick *laser;


int random_gen()
{
	int result;
	int max=255,min=1;
	back:
	result=rand()%((max+1-min)+min);// This is used to convert randomly generated integer to 1-255 range?
	if(result<1)
		goto back;
	else
		return(result);
}

int main(int argc, char **argv)
{
	// File IO
	FILE *xSlami,*ySlami,*xOdoi,*yOdoi,*xLmi,*yLmi,*xLmpi,*yLmpi,*abi,*rci,*lucni,*lcni,*rucni,*rcni,*mfai,*tsi;// Create txt files and erase previous data
	xSlami=fopen("xSlam.txt","w");
	fclose(xSlami);
	ySlami=fopen("ySlam.txt","w");
	fclose(ySlami);
	xOdoi=fopen("xOdo.txt","w");
	fclose(xOdoi);
	yOdoi=fopen("yOdo.txt","w");
	fclose(yOdoi);
	xLmi=fopen("xLm.txt","w");
	fclose(xLmi);
	yLmi=fopen("yLm.txt","w");
	fclose(yLmi);
	xLmpi=fopen("xLmp.txt","w");
	fclose(xLmpi);
	yLmpi=fopen("yLmp.txt","w");
	fclose(yLmpi);
	abi=fopen("ab.txt","w");
	fclose(abi);
	rci=fopen("rc.txt","w");
	fclose(rci);
	mfai=fopen("mfa.txt","w");
	fclose(mfai);
	tsi=fopen("ts.txt","w");
	fclose(tsi);
	rucni=fopen("rucn.txt","w");
	fclose(rucni);
	rcni=fopen("rcn.txt","w");
	fclose(rcni);
	lucni=fopen("lucn.txt","w");
	fclose(lucni);
	lcni=fopen("lcn.txt","w");
	fclose(lcni);

	//--------Declare and initialize  all variables used

	//------AIS variables
	int trip=0,fSF=250,bSF=150,ranCheck;
	long int matchgb[10][8],f[10][10],oaabs[10],oaabsbin[10][8],oaantibody[10][8],d[10],temp121[8],hpriorab[8],antibody[10][8],ab[10],antigen[8],match[10][10],/*matchoa[10],*/lm[4]={0,0,0,0},rm[4]={0,0,0,0},/*matchagab[10],*/matchabab[10][10],matchgg[10][10][8];
	double *cR,oamaxconc[10],abconc[10]={0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01},affinity2[10]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},a[11],da[11],affinity[10][10];
	long int hprior,critic,np=0,other,winner,final_antibody,abmax=0,winab=0,j,k,y,agn,u,v,p,q,r,oaabs2;
	double lmvP=0,rmvP=0,multiFactor,ct,eta=10.0,my_gamma,deltaAis=0.5,lmv=0.0,rmv=0.0,lmvUc=0.0,rmvUc=0.0,tempAis=0.0,temp1=0.0,e=2.7183,abmij,abmki,term1,term2,term3,term4;
	long int anomaly=0,itrn;

	cout<<"Press Enter to activate the robot!"<<"\n";
	cin.get();														// Press Enter to start the robot

	JACKRABBIT:														// Jump point for runtime error encounters and direction
	try																// change.
	{
		while(1)
		{
			agn=0,u=0,v=0,p=0,q=0,r=256;

			//------- Get Laser Data
			ArUtil::sleep(100);
			laser->lockDevice();
			readings=laser->getRawReadings();						// Get radings list from the LRF
			i = -1;
			for (it = readings->begin(); it != readings->end(); it++)
			{
				i++;
				scan[i]=(*it)->getRange();							// Store reading into an array starting at -90(LRF right)
			}
			laser->unlockDevice();
			//------------------------------

		  //--------AIS Algorithm begins-------------
			for(i=0;i<8;i++)
			{
				antigen[i]=0;
			}

			if(trip>=100)
			{
				trip=0;												// Reset to 0 after 100 reverse cycles
			}
			if(trip>=1)												// going backwards i.e. trip>=1.
			{
				for(i=0;i<=7;i++)
				{
					antigen[i]=read_back(&robot,i);					// Read all back sensors and store in antigen array
				}
				if(antigen[3]==1 && antigen[4]==1)
				{
					trip=0;											// go forwards if sensors pickup obstacle ahead of robot
					goto JACKRABBIT;								// i.e. trip=0 if antigen array is 00011000.
				}
			}
			if(trip==0)												// going forwards i.e. trip=0.
			{

				for(i=0;i<=7;i++)
				{
					antigen[i]=read(&robot,i);						// Read all front sensors and store in antigen array
				}
				if(antigen[2]==1 && antigen[3]==1 && antigen[4]==1 && antigen[5]==1)
				{
					ranCheck=rand()%2;											// turn and check if surroundings are clear else go back
					if(ranCheck==0)
					{
						robot.stop();
						robot.setDeltaHeading(90.00);
						ArUtil::sleep(1100);
						laser->lockDevice();
						readings=laser->getRawReadings();						// Get radings list from the LRF
						i = -1;
						for (it = readings->begin(); it != readings->end(); it++)
						{
							i++;
							scan[i]=(*it)->getRange();							// Store reading into an array starting at -90(LRF right)
						}
						laser->unlockDevice();
						for(i=0;i<=7;i++)
						{
							antigen[i]=read(&robot,i);							// Read all front sensors and store in antigen array
						}
						if(antigen[2]==1 && antigen[3]==1 && antigen[4]==1 && antigen[5]==1)
						{
							robot.stop();
							robot.setDeltaHeading(-180.00);
							ArUtil::sleep(1100);
							laser->lockDevice();
							readings=laser->getRawReadings();					// Get readings list from the LRF
							i = -1;
							for (it = readings->begin(); it != readings->end(); it++)
							{
								i++;
								scan[i]=(*it)->getRange();						// Store reading into an array starting at -90(LRF right)
							}
							laser->unlockDevice();
							for(i=0;i<=7;i++)
							{
								antigen[i]=read(&robot,i);						// Read all front sensors and store in antigen array
							}
							if(antigen[2]==1 && antigen[3]==1 && antigen[4]==1 && antigen[5]==1)
							{
								robot.setDeltaHeading(90);
								ArUtil::sleep(1000);
								goto cantAvoidRev;
							}
							else
								goto avoidRev;
						}
						else
							goto avoidRev;
					}
					else if(ranCheck=1)
					{
						robot.stop();
						robot.setDeltaHeading(-90);
						ArUtil::sleep(1100);
						laser->lockDevice();
						readings=laser->getRawReadings();						// Get readings list from the LRF
						i = -1;
						for (it = readings->begin(); it != readings->end(); it++)
						{
							i++;
							scan[i]=(*it)->getRange();							// Store reading into an array starting at -90(LRF right)
						}
						laser->unlockDevice();
						for(i=0;i<=7;i++)
						{
							antigen[i]=read(&robot,i);							// Read all front sensors and store in antigen array
						}
						if(antigen[2]==1 && antigen[3]==1 && antigen[4]==1 && antigen[5]==1)
						{
							robot.stop();
							robot.setDeltaHeading(180);
							ArUtil::sleep(1100);
							laser->lockDevice();
							readings=laser->getRawReadings();					// Get radings list from the LRF
							i = -1;
							for (it = readings->begin(); it != readings->end(); it++)
							{
								i++;
								scan[i]=(*it)->getRange();						// Store reading into an array starting at -90(LRF right)
							}
							laser->unlockDevice();
							for(i=0;i<=7;i++)
							{
								antigen[i]=read(&robot,i);						// Read all front sensors and store in antigen array
							}
							if(antigen[2]==1 && antigen[3]==1 && antigen[4]==1 && antigen[5]==1)
							{
								robot.setDeltaHeading(-90);
								ArUtil::sleep(1000);
								goto cantAvoidRev;
							}
							else
								goto avoidRev;
						}
						else
							goto avoidRev;
					}
					cantAvoidRev:
					trip=1;											// go backwards if sensors pickup obstacle ahead of robot
					goto JACKRABBIT;								// i.e. trip=1 if antigen array is 00111100.
				}
			}

			avoidRev:
			for(i=0;i<8;i++)
			{
				temp121[i]=0;
			}
			if(antigen[0]==0 && antigen[1]==0 && antigen[2]==0 && antigen[3]==0 && antigen[4]==0 && antigen[5]==0 && antigen[6]==0 && antigen[7]==0)
			{														// in this case, the robot is required to move forward
																	// at full speed.
				lmv=15;												//left motor velocity factor *7
				rmv=15;												//right motor velocity factor*7
				goto end2;
			}
			else
			{
				for(i=0;i<8;i++)									// Compliment of antigen
				{
					temp121[i]=!antigen[i];
				}
				u=0;
				v=0;
				u=((temp121[0]*1)+(temp121[1]*2)+(temp121[2]*4)+(temp121[3]*8));
				v=((temp121[7]*1)+(temp121[6]*2)+(temp121[5]*4)+(temp121[4]*8));
				for(i=0;i<10;i++)									// This for-loop is used to assign values of 0 into all
				{													// the matrices outside the iterations loop.
					da[i]=0.0;
					oaabs[i]=0;
					oamaxconc[i]=0.0;
					for(j=0;j<8;j++)
					{
						oaantibody[i][j]=0;
					}
				}
			//-----------------Create 10 iterations containing 10 random antibodies in each iteration-------------------
				itrn=1;
				while(itrn<=10)
				{
					for(i=0;i<10;i++)								// This for-loop is used to assign values of 0 into all
					{												// the matrices used in the iterations loop.
						for(j=0;j<10;j++)
						{
							matchabab[i][j]=0;
							affinity[i][j]=0.0;
							f[i][j]=0;
							for(k=0;k<8;k++)
							{
								matchgg[i][j][k]=0;
								matchgb[i][k]=0;
								antibody[i][k]=0;
								hpriorab[k]=0;
							}
						}
					}
					for(i=0;i<10;i++)								// This loop creates 10 antibodies and puts them in ab[]
					{
						y=0;
						ab[i]=random_gen();							// ab holds the integer value of the random antibody
						y=ab[i];									// generated.
						j=0;
						while(y!=0)
						{
							antibody[i][j++]=y%2;					// converts the integer value of the antibody into 8 bit
							y/=2;									// binary antibody[][] matrix, each row contains 8 bits
						}											// of an antibody generated
					}
					for(i=0;i<10;i++)								// Check how antigen and the ramdomly generated antibody
					{												// compliment each other.
						for(j=0;j<8;j++)
						{
							matchgb[i][j]=antigen[j] ^ antibody[i][j];
						}
					}
					for(i=0;i<10;i++)
					{
						d[i]=0;
					}
					for(i=0;i<10;i++)								// d[] gives a numerical value to the compliment between
					{												// antibodies and antigens.
						for(j=0;j<8;j++)
						{
							d[i]+=matchgb[i][j];
						}
						if(d[i]==8)
						{
							rmv=((antibody[i][0]*1)+(antibody[i][1]*2)+(antibody[i][2]*4)+(antibody[i][3]*8));// Sensor locations appear to be multiplied by a power of 2 based upon
							lmv=((antibody[i][7]*1)+(antibody[i][6]*2)+(antibody[i][5]*4)+(antibody[i][4]*8));// the side the sensor is on, and increasing from rear to front of the robot
							goto end1;
						}
					}
					for(i=0;i<10;i++)
					{
						if(d[i]>=5)									// if more than 5 bits match, the affinity is increased.
						{											// else it remains 0.
							affinity2[i]=(0.1*d[i]);
						}
						else
						{
							affinity2[i]=(0.0);
						}
					}
					for(i=0;i<10;i++)								// Check how the ramdomly generated antibody compliment
					{												// each other to check if they are same.
						if(d[i]>=5)
						{
							for(j=0;j<10;j++)
							{
								for(k=0;k<8;k++)
								{
									matchgg[i][j][k]=antibody[i][k] ^ antibody[j][k];
								}
							}
						}
					}
					for(i=0;i<10;i++)								// f[][] gives a numerical value to the compliment
					{												// between two antibodies; counts the number of 1s in
						for(j=0;j<10;j++)							// the compliment matrix matchgg[][][].
						{
							f[i][j]=0;
						}
					}
					for(i=0;i<10;i++)
					{
						for(j=0;j<10;j++)
						{
							for(k=0;k<8;k++)
							{
								f[i][j]+=matchgg[i][j][k];
							}
						}
					}
					for(i=0;i<10;i++)
					{
						for(j=0;j<10;j++)
						{
							if(f[i][j]>5)
							{
								affinity[i][j]=(0.1*f[i][j]);
							}
							else
							{
								affinity[i][j]=(0.0);
							}
						}
					}

				//------------Dynamic equation by Jerne(rate of change of concentration of each antibody)------------
					for(i=0;i<10;i++)
					{
						abconc[i]=0.01;
						a[i]=0;
						for(j=0;j<10;j++)
						{
							abmij=0.0;
							abmki=0.0;
							for(int k=0;k<10;k++)
							{
								if(affinity2[i]>0.5)
								{
									abmij+=(affinity2[i]*abconc[k]);
								}
								if(affinity2[i]<0.5)
								{
									abmki+=(affinity2[i]*abconc[k]);
								}
								term1=abmij;
								term2=abmki;
								term3=affinity[j][k];
								term4=0.01;
								da[i]=((term1-term2+term3-term4)*abconc[i]);
																	// Jerne's concentration equation
								//----------confusion in this part
								a[i]=(da[i]+(da[i]*0.5));// how to get reverse derivative of equation 4.4
								//--------------------------------
								abconc[i]=(1/(1+pow(e,(0.5-a[i]))));// Jerne's squashing function equation *0.05
							}
						}
					}

				//--------------Finding Antibody with max concentration----
					temp1=0.0;
					temp1=abconc[0];
					for(i=0;i<10;i++)
					{
						if(temp1<abconc[i])
						{
							temp1=abconc[i];
						}
					}
					abmax=0;
					for(i=0;i<10;i++)
					{
						if(temp1==abconc[i])
						{
							abmax=i;
							break;
						}
					}
					for(i=0;i<8;i++)
					{
						hpriorab[i]=0;
					}
					for(i=0;i<8;i++)
					{
						hpriorab[i]=!antigen[i];
					}
					for(i=0;i<8;i++)
					{
						hprior+=(hpriorab[i]*2^i);					// Gets the integer value of the perfect antibody.
					}
					if(ab[abmax]==hprior)							// Check if antibody with max concentration is the one
					{												// with high priority
						critic=0;
					}
					else
					{
						critic=1;
					}
					switch(critic)
					{
						case 0: ct=0.5;
							break;

						case 1: np++;
							ct=(1/(1+pow(e,(-eta*np))));
							break;
					}
					if(ct>deltaAis)
					{
						my_gamma=0.01;
					}
					else
					{
						my_gamma=0.0;
					}
					winner=abmax;
					if(critic==0)
					{
						for(j=0;j<10;j++)
						{
							other=ab[j];
							if(ab[j]!=ab[winner])
							{
								affinity[winner][other]=(1/(1+(pow(e,(0.5-((affinity[winner][other])*(1+my_gamma)))))));
							}
						}
					}
					oaabs[itrn]=ab[winner];
					for(i=0;i<8;i++)								// Convert to binary form
					{
						y=0;
						y=oaabs[itrn];
						while(y!=0)
						{
							oaabsbin[itrn-1][i++]=y%2;
							y/=2;
						}
					}
					oamaxconc[itrn]=abconc[winner];
					for(i=0;i<8;i++)
					{
						oaantibody[itrn][i]=antibody[winner][i];
					}
					itrn++;
					cout.flush();
	  			}
			//-----------10 iteration while loop ends here--------

				winab=winner;
				for(i=0;i<10;i++)
				{
					for(j=0;j<8;j++)
					{
						match[i][j]=0;
					}
				}
				for(i=0;i<10;i++)
				{
					for(j=0;j<8;j++)
					{
						match[i][j]=antigen[j] ^ oaabsbin[i][j];	// Check how antigen and the ramdomly generated antibody
					}												// compliment each other.
				}
				for(i=0;i<10;i++)
				{
					d[i]=0;
				}
				for(i=0;i<10;i++)
				{
					for(j=0;j<8;j++)
					{
						d[i]+=match[i][j];
					}
				}
				double tempo=oamaxconc[0];
				for(i=0;i<10;i++)
				{
					if(tempo<oamaxconc[i])
					{
						 tempo=oamaxconc[i];
					}
				}
				int x=0;
				for(i=0;i<10;i++)
				{
					if(tempo==oamaxconc[i])
					{
						x=i;
						break;
					}
				}

				final_antibody=0;
				winab=x;
	//-----------AIS Ends here----------------------

	//------------Wheel voltage conversion----------
				for(i=0;i<4;i++)
				{
					lm[i]=oaantibody[winab][i];
				}
				for(i=4;i<8;i++)
				{
					rm[i-4]=oaantibody[winab][i];
				}
				for(i=0;i<4;i++)
				{
					rmv=((lm[0]*1)+(lm[1]*2)+(lm[2]*4)+(lm[3]*8));	// Motor velocity formulas. Note that the array locations
					lmv=((rm[3]*1)+(rm[2]*2)+(rm[1]*4)+(rm[0]*8));	// are multiplied by their corresponding power of 2
				}
				oaabs2=lm[0]*1+lm[1]*2+lm[2]*4+lm[3]*8+rm[0]*16+rm[1]*32+rm[2]*64+rm[3]*128;

	//----------Fail Safe-----------------
				end1:
				if(v>u)
				{
					if(lmv>rmv)
					{
						goto end2;
					}
					else if(lmv<=rmv)								// If AIS computes wrong antibody which will command
					{												// the robot to turn in direction opposite to anticipated
						goto retry1;								// o/p then retry and find new antibodythe robot to turn
					}												// in direction opposite to anticipated o/p then retry
				}													// and find new antibody.
				else if(u>v)
				{
					if(rmv>lmv)
					{
						goto end2;
					}
					else if(rmv<=lmv)
					{
						goto retry1;
					}
				}
				end2:
				lmv=lmv/15;		// Normalize the velocity to 1
				rmv=rmv/15;
				lmvUc=lmv*fSF;	// Scale normalized velocity to maximum allowable speed specified
				rmvUc=rmv*fSF;
				if(trip>=1)
				{
					rmvUc=lmv*bSF;
					lmvUc=rmv*bSF;
				}

		//---------ACR--------
				multiFactor=zFunc(range[181],600,2500);
				cR=acr(&robot,controlResponse,lmvUc,rmvUc,multiFactor,lmvP,rmvP);

				if(trip>=1)
				{
					robot.setVel2(lmvUc*-1,rmvUc*-1);
				}
				else
				{
					robot.setVel2(cR[0],cR[1]);
				}

				lmvP=cR[0];
				rmvP=cR[1];
		//----------------------

				if(trip>=1)
				{
					trip++;
				}
				retry1:
				ofstream rc("rc.txt",ios::app);
				rc<<runCount<<"	";
				rc.close();
				ofstream lucn("lucn.txt",ios::app);
				lucn<<lmvUc<<"	";
				lucn.close();
				ofstream lcn("lcn.txt",ios::app);
				lcn<<cR[0]<<"	";
				lcn.close();
				ofstream rucn("rucn.txt",ios::app);
				rucn<<rmvUc<<"	";
				rucn.close();
				ofstream rcn("rcn.txt",ios::app);
				rcn<<cR[1]<<"	";
				rcn.close();
				ofstream mfa("mfa.txt",ios::app);
				mfa<<multiFactor<<"	";
				mfa.close();
				ofstream ab("ab.txt",ios::app);
				ab<<oaabs2<<"	"<<oamaxconc[winab]<<"\n";
				ab.close();
				oaabs2=0;
	//-----------------------------------
			}
			runCount++;
			DbCheckCount++;						// Landmark DB check after count reaches LMCheck cycles
			system("CLS");
			cout<<"# of anomalies detected ("<<anomaly<<")"<<endl<<endl;
			cout<<"runcount= "<<runCount<<endl;
			cout<<"Number of landmarks in database= "<<DbCheckCount<<endl;
		}
	}

	catch (...)
	{
		anomaly++;
		{
			goto JACKRABBIT;
		}
	}
	robot.waitForRunExit();
	laser->lockDevice();
	laser->stopRunning();
	laser->disconnect();
	laser->unlockDevice();
	Aria::shutdown();
	return 0;													// main() ends, Program exits
}
