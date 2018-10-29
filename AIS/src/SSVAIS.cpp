// SLAM and Robot Navigation with Artificial Immune System using Speed Variable Obstacle Detection
// Coded by:- Pranav Phadke
// Revision:- 4.f
// Date created/modified:- 04/30/2013

#include"Aria.h" 
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
#include<Eigen/Dense>
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

double *acr(ArRobot *thisRobot,double cr[2],double lmvUc,double rmvUc,double mF, double lmvCp,double rmvCp)
{
	cr[0]=lmvCp+(mF*(lmvUc-lmvCp));
	cr[1]=rmvCp+(mF*(rmvUc-rmvCp));
	return cr;
}
double zFunc(double bM,int a=1500,int b=2500)
{
	double mF, iT;
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
	return mF;
}
double *aes(ArRobot *thisRobot, double r[182], int bMax=2000)
{
	double speed=abs(thisRobot->getVel()),b=0;		// Get speed from the robot microprocessor and set
	long int m=100,c=80,a=380;						// c=80,a=380,m=100
	b=bMax/(1+exp((m-speed)/c));					// Sigmoidal function equation with speed as variable
	for(int i=0;i<181;i++)
	{
		r[i]=abs((a*b)/sqrt(pow((b*cos(i*PI/180)),2)+pow((a*sin(i*PI/180)),2)));// Elliptical threshold function r(theta)=ab/sqrt[(bcos(theta))^2+(asin(theta))^2]
	}
	r[181]=b;
	return r;										// Return AES threshold array
}
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
	Aria::init();													// Initialize the ARIA library 
	ArSimpleConnector connector(&argc, argv);						// Instantiate a connector 
	ArRobot robot;													// Instantiate robot 	
	laser= new ArSick;												// Create object of ArSick
	ArSonarDevice sonar;											// Instantiate sonar
	robot.addRangeDevice(&sonar);									// Add sonar to the robot
	robot.addRangeDevice(laser);									// Add LRF to the robot
	ArKeyHandler keyHandler;										// Used to perform actions when keyboard key is pressed
	Aria::setKeyHandler(&keyHandler);								// For e.g. to exit user can press the esc key anytime
	robot.attachKeyHandler(&keyHandler);
	printf("Press Esc to exit program!\n");

	//-------Connection to robot---------
	if (!connector.parseArgs())										// Parse arguments to the connector 
	{
		printf("Unable to identify default settings \n");			// Exit if there is an error 
		Aria::exit(0); 
		exit(1); 
	}
	if (!connector.connectRobot(&robot))							// Connect to the robot 
	{
		printf("Unable to connect to robot \n");					// Exit if there is an error 
		Aria::exit(0);
		exit(1);
	}
	robot.runAsync(true);											// Run robot in asynchronous mode 
	robot.comInt(ArCommands::ENABLE, 1);							// Turn on the motors 

	//--------Connect to LRF------------
	connector.setupLaser(laser);
	laser->runAsync();												// Run LRF in asynchronous mode
	if(!laser->blockingConnect())									// Connect to LRF
	{
		cout<<"Unable to connect to Sick LRF\n";					// Exit if there is an error
		Aria::exit(0);
		exit(1);
	}
	const std::list<ArSensorReading *> *readings;					// Instantiate Laser readings list
	std::list<ArSensorReading *>::const_iterator it;				// Instantiate iterator for traversing the readings list
	
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

	cout<<"Press Enter to activate the robot!"<<"\n";
	cin.get();														// Press Enter to start the robot
	
	//--------Declare and initialize  all variables used

	//------AIS variables
	int trip=0,fSF=250,bSF=150,ranCheck;
	long int matchgb[10][8],f[10][10],oaabs[10],oaabsbin[10][8],oaantibody[10][8],d[10],temp121[8],hpriorab[8],antibody[10][8],ab[10],antigen[8],match[10][10],/*matchoa[10],*/lm[4]={0,0,0,0},rm[4]={0,0,0,0},/*matchagab[10],*/matchabab[10][10],matchgg[10][10][8];
	double *cR,oamaxconc[10],abconc[10]={0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01},affinity2[10]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},a[11],da[11],affinity[10][10];
	long int hprior,critic,np=0,other,winner,final_antibody,abmax=0,winab=0,j,k,y,agn,u,v,p,q,r,oaabs2;
	double lmvP=0,rmvP=0,multiFactor,ct,eta=10.0,my_gamma,deltaAis=0.5,lmv=0.0,rmv=0.0,lmvUc=0.0,rmvUc=0.0,tempAis=0.0,temp1=0.0,e=2.7183,abmij,abmki,term1,term2,term3,term4;
	long int anomaly=0,itrn;

	//------Line Extraction variables
	double minDist,minDist2Lm[18][2],secRanX[100][2],secRanY[100][2],secRanLine[100][2],consensusSet[100][10][2],lineConsensus[18][100],secLineM[18][1],secLineC[18][1],secLineX[18][10][1],secLineY[18][10][1],scanLmRf[18][1][2],scanLm[18][8],scanSecX[18][10][1],scanSecY[18][10][1],LmDb[3000][10],temp[3000][10],ZtObs[7][18],ZtPredict[7][18],dist2Pt,dist2Lm,sumX,sumY,sumX2,sumXY;
	int minDistID,lineThreshold=10,checkRd=10,scanLmId=1,minOcc=5,counter=0,lmCheck=300,sectors=18,sectorSize=10,LmDbSize=3000,DbPt=0,lineItr=100,linePt=2,numCount,DbCheckCount=0,runCount=0,random,countPt,setIndex,max,maxID,m,LmROb,l;

	//------SLAM variables
	double cp=0.0001,rc=0.005,bd=0.5,cmr=0.001,cmb=0;
	MatrixXd X(7,1),GtEG(3,3),RtEG(3,3),QtEG(2,2),P(7,7),KtEG(7,2),HtEG(2,7),Innov(2,1);
	MatrixXd Pnn(2,2),Jxr(2,3),Jz(2,2),Pri(3,2),Prn(3,2),Prr(3,3),Pni(2,2),ZtObsEGs(2,1),ZtPredictEGs(2,1),ZtPredictEG(3,18),ZtObsEG(3,18);
	Matrix2d KtInnerTerm;
	double deltaTrans,deltaX,deltaY,deltaTh,robotXHist=0.0,robotYHist=0.0,robotThHist=0.0;


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
			
		//--------Landmark Extraction starts
		// Get robot position from odometry
		robot.lock();
		robotX=robot.getX();
		robotY=robot.getY();
		robotTh=robot.getTh();
		robot.unlock();
		cout<<robotTh<<endl;
			
		// Convert range and bearing to cartesian
		for (i=0;i<180;i++)
		{
			scanX[i]=scan[i]*cos((i-90)*(PI/180));
			scanY[i]=scan[i]*sin((i-90)*(PI/180));
		}

		// Divide scan into 18 sectors 10 degrees each
		for (i=0;i<sectors;i++)
		{
			for (j=0;j<sectorSize;j++)
			{
				scanSecX[i][j][0]=scanX[((i)*sectorSize)+j];
				scanSecY[i][j][0]=scanY[((i)*sectorSize)+j];
			}
		}

		// LSL Algorithm
		for(i=0;i<sectors;i++)
		{
			// Get 100 LSLines
			for(k=0;k<lineItr;k++)
			{
				// Get 2 random readings from the sector. More than two points may be used.
				sumX=0.0;sumY=0.0;sumXY=0.0;sumX2=0.0;
				for(numCount=0;numCount<linePt;numCount++)
				{
					random=rand()%10;
					secRanX[k][numCount]=scanSecX[i][random][0];
					secRanY[k][numCount]=scanSecY[i][random][0];

				// Get variables for equation used in calculation of LSL parameters m and c            
					sumX=sumX+secRanX[k][numCount];
					sumY=sumY+secRanY[k][numCount];
					sumXY=sumXY+(secRanX[k][numCount]*secRanY[k][numCount]);
					sumX2=sumX2+(secRanX[k][numCount]*secRanX[k][numCount]);
				}
					
				// Get least square line fitting these 2 random points        
				secRanLine[k][1]=((sumY*sumX2)-(sumX*sumXY))/((2*sumX2)-(sumX*sumX));// c
				secRanLine[k][0]=((2*sumXY)-(sumX*sumY))/((2*sumX2)-(sumX*sumX));// m
					
				// Get distance from these 10 lines to all the points in the sector and find consensus set
				countPt=0;setIndex=0;
				for(j=0;j<sectorSize;j++)
				{
					dist2Pt=abs(scanSecY[i][j][0]-(secRanLine[k][0]*scanSecX[i][j][0])-secRanLine[k][1])/sqrt((secRanLine[k][0]*secRanLine[k][0])+1);
					if(dist2Pt<=lineThreshold)
					{
						countPt=countPt+1;
						setIndex=setIndex+1;
						consensusSet[k][setIndex][0]=scanSecX[i][j][0];
						consensusSet[k][setIndex][1]=scanSecY[i][j][0];
					}
				}
				lineConsensus[i][k]=countPt;
			}
				
			// Get line with maximum consensus    
			max=0;maxID=0;
			for(k=0;k<lineItr;k++)
			{
				if(lineConsensus[i][k]>max)
				{   
					max=lineConsensus[i][k];
					maxID=k;
				}
			}
				
			// Get variables for equation used in calculation of LSL parameters m and c
			secLineM[i][0]=secRanLine[maxID][0];
			secLineC[i][0]=secRanLine[maxID][1];
			for(j=0;j<sectorSize;j++)
			{
				secLineY[i][j][0]=scanSecY[i][j][0];
				secLineX[i][j][0]=(scanSecY[i][j][0]-secLineC[i][0])/secLineM[i][0];
			}
				
			// Averaging extremities. Used to smoothen the extracted curve    
			if((i>=1)&&(i<(sectors-1)))
			{
				secLineY[i][0][0]=(secLineY[i][0][0]+secLineY[i-1][sectorSize-1][0])/2;
				secLineX[i][0][0]=secLineX[i-1][sectorSize-1][0];
			} 

			if(i==0)
			{    
				secLineY[i][0][0]=0;
			}
 
			if(i==(sectors-1))
			{
				secLineY[i][0][0]=(secLineY[i][0][0]+secLineY[i-1][sectorSize-1][0])/2;
				secLineX[i][0][0]=secLineX[i-1][sectorSize-1][0];
				secLineY[i][sectorSize-1][0]=0;
			}
		}

		// Get the middle point on the line and take that as the landmark
		for(i=0;i<sectors;i++)
		{
			scanLmRf[i][0][1]=secLineY[i][4][0];//-- Get y coordinates of the midway point on the line
			scanLmRf[i][0][0]=secLineX[i][4][0];//-- Get x coordinate of the midway point on the line
		}
	
		// Transform these points to world frame and put into scan landmark array 
		// scanLm(M,C,xLinePt1,yLinePt1,xLinePt2,yLinePt2,xPointLm,yPointLm)->dimensions ix8
		for(i=0;i<sectors;i++)
		{
			scanLm[i][0]=secLineM[i][0];
			scanLm[i][1]=secLineC[i][0];
			scanLm[i][2]=secLineX[i][0][0]*cos(robotTh*PI/180)-secLineY[i][0][0]*sin(robotTh*PI/180)+robotX;
			scanLm[i][3]=secLineY[i][0][0]*cos(robotTh*PI/180)+secLineX[i][0][0]*sin(robotTh*PI/180)+robotY;
			scanLm[i][4]=secLineX[i][sectorSize-1][0]*cos(robotTh*PI/180)-secLineY[i][sectorSize-1][0]*sin(robotTh*PI/180)+robotX; 
			scanLm[i][5]=secLineY[i][sectorSize-1][1]*cos(robotTh*PI/180)+secLineX[i][sectorSize-1][0]*sin(robotTh*PI/180)+robotY;
			scanLm[i][6]=scanLmRf[i][0][0]*cos(robotTh*PI/180)-scanLmRf[i][0][1]*sin(robotTh*PI/180)+robotX;
			scanLm[i][7]=scanLmRf[i][0][1]*cos(robotTh*PI/180)+scanLmRf[i][0][0]*sin(robotTh*PI/180)+robotY;
		}

		// Create global landmark database
		// LmDb(id,seen count,M,C,xLinePt1,yLinePt1,xLinePt2,yLinePt2,xPointLm,yPointLm)
		// Landmark DB maintenance. Removes landmarks not seen often
		if((DbCheckCount>lmCheck) || (DbPt>2999))
		{
			// copy LmDb with seen count more than minOcc to temporary array and delete original landmark db
			for(m=0;m<DbPt;m++)
			{
				if(LmDb[m][0]!=0)
				{
					if(LmDb[m][1]>=minOcc)
					{
						temp[counter][0]=LmDb[m][0];
						temp[counter][1]=LmDb[m][1];
						temp[counter][2]=LmDb[m][2];
						temp[counter][3]=LmDb[m][3];
						temp[counter][4]=LmDb[m][4];
						temp[counter][5]=LmDb[m][5];
						temp[counter][6]=LmDb[m][6];
						temp[counter][7]=LmDb[m][7];
						temp[counter][8]=LmDb[m][8];
						temp[counter][9]=LmDb[m][9];
						ofstream xLm("xLm.txt",ios::app);
						xLm<<LmDb[m][8]<<"	";
						xLm.close();
						ofstream yLm("yLm.txt",ios::app);
						yLm<<LmDb[m][9]<<"	";
						yLm.close();
						LmDb[m][0]=0;						// Delete ids from original db
						counter++;							// Pointing to next free location in the db
					}
				}
			}
			DbPt=counter;
			// now copy temp back to landmark db
			for(m=0;m<DbPt;m++)
			{
				LmDb[m][0]=temp[m][0];
				LmDb[m][1]=temp[m][1];
				LmDb[m][2]=temp[m][2];
				LmDb[m][3]=temp[m][3];
				LmDb[m][4]=temp[m][4];
				LmDb[m][5]=temp[m][5];
				LmDb[m][6]=temp[m][6];
				LmDb[m][7]=temp[m][7];
				LmDb[m][8]=temp[m][8];
				LmDb[m][9]=temp[m][9];
			}
			counter=0;
			DbCheckCount=0;
		}

		// Check if extracted landmark is already present in the database
		LmROb=0;
		for(i=0;i<sectors;i++)
		{
			if(scanLmId<=18)								// Add whatever landmarks is extracted for the first 18 landmarks
			{
				LmDb[DbPt][0]=scanLmId;
				LmDb[DbPt][1]=1;
				LmDb[DbPt][2]=scanLm[i][0];
				LmDb[DbPt][3]=scanLm[i][1];
				LmDb[DbPt][4]=scanLm[i][2];
				LmDb[DbPt][5]=scanLm[i][3];
				LmDb[DbPt][6]=scanLm[i][4];
				LmDb[DbPt][7]=scanLm[i][5];
				LmDb[DbPt][8]=scanLm[i][6];
				LmDb[DbPt][9]=scanLm[i][7];
				goto skipAssociation;
			}
				
			for(j=0;j<18;j++)
			{
				minDist2Lm[j][0]=minDist2Lm[j][1]=0.0;
			}
			j=minDistID=0;
			dist2Lm=minDist=0.0;
			for(l=0;l<DbPt;l++)								// Try to associate; if not add to db
			{

				if(scanLm[i][6]>0 && LmDb[l][8]>0 && scanLm[i][7]>0 && LmDb[l][9]>0)					
				{
					// First get the distance of the extracted landmark from landmarks in the database
					dist2Lm=sqrt(pow((scanLm[i][6]-LmDb[l][8]),2)+pow((scanLm[i][7]-LmDb[l][9]),2));
					if (dist2Lm<=checkRd)
					{
						// Get the min distance from set of passed landmarks
						minDist2Lm[j][0]=dist2Lm;
						minDist2Lm[j][1]=l;
						j++;
					}
				}
				if(scanLm[i][6]>0 && LmDb[l][8]>0 && scanLm[i][7]<0 && LmDb[l][9]<0)					
				{
					// First get the distance of the extracted landmark from landmarks in the database
					dist2Lm=sqrt(pow((scanLm[i][6]-LmDb[l][8]),2)+pow((scanLm[i][7]-LmDb[l][9]),2));
					if (dist2Lm<=checkRd)
					{
						// Get the min distance from set of passed landmarks
						minDist2Lm[j][0]=dist2Lm;
						minDist2Lm[j][1]=l;
						j++;
					}
				}
				if(scanLm[i][6]<0 && LmDb[l][8]<0 && scanLm[i][7]>0 && LmDb[l][9]>0)					
				{
					// First get the distance of the extracted landmark from landmarks in the database
					dist2Lm=sqrt(pow((scanLm[i][6]-LmDb[l][8]),2)+pow((scanLm[i][7]-LmDb[l][9]),2));
					if (dist2Lm<=checkRd)
					{
						// Get the min distance from set of passed landmarks
						minDist2Lm[j][0]=dist2Lm;
						minDist2Lm[j][1]=l;
						j++;
					}
				}
				if(scanLm[i][6]<0 && LmDb[l][8]<0 && scanLm[i][7]<0 && LmDb[l][9]<0)					
				{
					// First get the distance of the extracted landmark from landmarks in the database
					dist2Lm=sqrt(pow((scanLm[i][6]-LmDb[l][8]),2)+pow((scanLm[i][7]-LmDb[l][9]),2));
					if (dist2Lm<=checkRd)
					{
						// Get the min distance from set of passed landmarks
						minDist2Lm[j][0]=dist2Lm;
						minDist2Lm[j][1]=l;
						j++;
					}
				}
			}
			// Now get the Landmark with least distance to the extracted feature
			minDist=minDist2Lm[0][0];
			minDistID=minDist2Lm[0][1];
			for(k=0;k<j;k++)
			{
				if(minDist2Lm[k][0]<minDist)
				{
					minDistID=minDist2Lm[k][1];
					minDist=minDist2Lm[k][0];
				}
			}
			if(j>0)											// Extracted landmark was associated
			{
				LmDb[minDistID][1]+=1;

				ZtPredict[0][LmROb]=sqrt(pow(LmDb[minDistID][8]-robotX,2)+pow(LmDb[minDistID][9]-robotY,2));// Range in RF
				ZtPredict[1][LmROb]=(atan2(LmDb[minDistID][9],LmDb[minDistID][8])*(180/PI))-robotTh;// Bearing in RF
				ZtPredict[2][LmROb]=LmDb[minDistID][0];// ID for the reobserved landmark in the DB
				ZtPredict[3][LmROb]=LmDb[minDistID][8];// X coordinate
				ZtPredict[4][LmROb]=LmDb[minDistID][9];// Y coordinate
				ZtPredict[5][LmROb]=sqrt(pow(LmDb[minDistID][8],2)+pow(LmDb[minDistID][9],2));// Range in WF
				ZtPredict[6][LmROb]=atan2(LmDb[minDistID][9],LmDb[minDistID][8])*(180/PI);// Bearing in WF

				ZtObs[0][LmROb]=sqrt(pow(scanLm[i][6]-robotX,2)+pow(scanLm[i][7]-robotY,2));// Observed range in RF
				ZtObs[1][LmROb]=(atan2(scanLm[i][7],scanLm[i][6])*(180/PI))-robotTh;// Bearing in RF
				ZtObs[2][LmROb]=0;
				ZtObs[3][LmROb]=scanLm[i][6];// X coordinate
				ZtObs[4][LmROb]=scanLm[i][7];// Y coordinate
				ZtObs[5][LmROb]=sqrt(pow(scanLm[i][6],2)+pow(scanLm[i][7],2));// Range in WF
				ZtObs[6][LmROb]=atan2(scanLm[i][7],scanLm[i][6])*(180/PI);// Bearing in WF
				LmROb++;
				goto passedAssociation;	
			}
			else if(j==0)									// If no associations found add the extracted landmark to the db
			{
				LmDb[DbPt][0]=scanLmId;
				LmDb[DbPt][1]=1;
				LmDb[DbPt][2]=scanLm[i][0];
				LmDb[DbPt][3]=scanLm[i][1];
				LmDb[DbPt][4]=scanLm[i][2];
				LmDb[DbPt][5]=scanLm[i][3];
				LmDb[DbPt][6]=scanLm[i][4];
				LmDb[DbPt][7]=scanLm[i][5];
				LmDb[DbPt][8]=scanLm[i][6];
				LmDb[DbPt][9]=scanLm[i][7];
				ofstream xLmp("xLmp.txt",ios::app);
				xLmp<<LmDb[DbPt][8]<<"	";
				xLmp.close();
				ofstream yLmp("yLmp.txt",ios::app);
				yLmp<<LmDb[DbPt][9]<<"	";
				yLmp.close();
				goto skipAssociation;
			}
				
			skipAssociation:
			scanLmId++;
			DbPt++;
			passedAssociation:;
		}

		std::cout<<"Re-Observed landmarks="<<LmROb<<endl;
		std::cout<<"Number of landmarks="<<DbPt<<endl;
		//--------Landmark Extraction ends
		
			// SLAM starts
			// Update state prediction and deltas
			deltaY=robotY-robotYHist;
			deltaX=robotX-robotXHist;
			deltaTh=robotTh-robotThHist;
			deltaTrans=sqrt(pow((robotX-robotXHist),2)+pow((robotY-robotYHist),2));// Translational change
			
			// Form State matrix X
			if(runCount<1)
			{
				Prr<<MatrixXd::Zero(3,3);
				Pri<<MatrixXd::Zero(3,2);
				Prn<<MatrixXd::Zero(3,2);
				X<<MatrixXd::Zero(7,1);
				P<<MatrixXd::Zero(7,7);
				HtEG<<MatrixXd::Zero(2,7);
				Prr(0,0)=Prr(1,1)=Prr(2,2)=Pri(0,0)=Pri(1,1)=1;		// Initialize with large uncertainty
			}
			
			// Update State Matrix X
			X(0,0)=robotX;
			X(1,0)=robotY;
			X(2,0)=robotTh;
		
			// Update Gt and Jxr
			for(i=0;i<3;i++)
			{
				for(j=0;j<3;j++)
				{
					if(i==j)
					{
						GtEG(i,j)=1;
					}
					else
					{
						GtEG(i,j)=0;
					}
				}
			}
			for(i=0;i<2;i++)
			{
				for(j=0;j<3;j++)
				{
					if(i==j)
					{
						Jxr(i,j)=1;
					}
					else
					{
						Jxr(i,j)=0;
					}
				}
			}
			Jxr(0,2)=GtEG(0,2)=-1*deltaY;
			Jxr(1,2)=GtEG(1,2)=deltaX;
			
			// Update process noise Rt
			RtEG(0,0)=cp*pow(deltaX,2);
			RtEG(1,1)=cp*pow(deltaY,2);	
			RtEG(2,2)=cp*pow(deltaTh,2);
			RtEG(0,1)=cp*deltaX*deltaY;
			RtEG(0,2)=cp*deltaX*deltaTh;
			RtEG(1,0)=cp*cp*deltaX*deltaY;
			RtEG(1,2)=cp*deltaTh*deltaY;
			RtEG(2,0)=cp*deltaX*deltaTh;
			RtEG(2,1)=cp*deltaTh*deltaY;
			
			// Update measurement noise Qt
			for(i=0;i<2;i++)
			{
				for(j=0;j<2;j++)
				{
					QtEG(i,j)=0;
				}
			}
			QtEG(0,0)=rc;											// Assume rc*100% range measurement error
			QtEG(1,1)=bd;											// Assume bd degree bearing error
			
			// Get Jacobian Jz
			Jz(0,0)=cos(robotTh);
			Jz(0,1)=-deltaTrans*sin(robotTh);
			Jz(1,0)=sin(robotTh);
			Jz(1,1)=deltaTrans*cos(robotTh);
			
			// Get sub-matrices and put them in P
			Prr=(GtEG*Prr*GtEG.transpose())+RtEG;					// Get Prr
			Pri=GtEG*Pri;											// Get Pri
			Prn=Prr*Jxr.transpose();								// Get Prn
			Pnn=(Jxr*Prr*Jxr.transpose())+(Jz*QtEG*Jz.transpose());	// Get Pnn
			Pni=Jxr*Pri;											// Get Pni
			
			P(0,0)=Prr(0,0);
			P(0,1)=Prr(0,1);
			P(0,2)=Prr(0,2);
			P(1,0)=Prr(1,0);
			P(1,1)=Prr(1,1);
			P(1,2)=Prr(1,2);
			P(2,0)=Prr(2,0);
			P(2,1)=Prr(2,1);
			P(2,2)=Prr(2,2);
			
			// Put Pri and Pir in P
			P(3,0)=P(0,3)=Pri(0,0);
			P(3,1)=P(1,3)=Pri(1,0);
			P(3,2)=P(2,3)=Pri(2,0);
			P(4,0)=P(0,4)=Pri(0,1);
			P(4,1)=P(1,4)=Pri(1,1);
			P(4,2)=P(2,4)=Pri(2,1);

			// Put Pnn in P for first re-observed landmark in DB
			P(3,3)=Pnn(0,0);
			P(3,4)=Pnn(0,1);
			P(4,3)=Pnn(1,0);
			P(4,4)=Pnn(1,1);			
			
			// Initialize H
			HtEG(1,2)=-1;

			// Put Prn and Pnr in P
			P(5,0)=P(0,5)=Prn(0,0);
			P(5,1)=P(1,5)=Prn(1,0);
			P(5,2)=P(2,5)=Prn(2,0);
			P(6,0)=P(0,6)=Prn(0,1);
			P(6,1)=P(1,6)=Prn(1,1);
			P(6,2)=P(2,6)=Prn(2,1);

			// Put Pnn in P for 2nd and n landmarks in DB
			P(5,5)=Pnn(0,0);
			P(5,6)=Pnn(0,1);
			P(6,5)=Pnn(1,0);
			P(6,6)=Pnn(1,1);

			// Put Pni
			P(3,5)=P(5,3)=Pni(0,0);
			P(4,5)=P(5,4)=Pni(0,1);
			P(3,6)=P(6,3)=Pni(1,0);
			P(4,6)=P(6,4)=Pni(1,1);

			// Do following for all re-observed landmarks
			for(i=0;i<LmROb;i++)
			{
				// Update re-observed landmark's predicted position
				ZtPredictEGs(0,0)=ZtPredictEG(0,i)=ZtPredict[0][i];	// Get range of the associated landmark from the robot
				ZtPredictEGs(1,0)=ZtPredictEG(1,i)=ZtPredict[1][i];	// Get bearing of the associated landmark from the robot
				
				// Get ZtObs which contains actual parameters for the current association pair
				ZtObsEGs(0,0)=ZtObsEG(0,i)=ZtObs[0][i];				// Actual range of the landmark under observation
				ZtObsEGs(1,0)=ZtObsEG(1,i)=ZtObs[1][i];				// Actual bearing of the landmark under observation

				// Update measure model jacobian Ht for the current re-observed landmark
				HtEG(0,0)=(robotX-ZtPredict[3][i])/ZtPredict[5][i];
				HtEG(0,1)=(robotY-ZtPredict[4][i])/ZtPredict[5][i];
				HtEG(1,0)=(ZtPredict[4][i]-robotY)/pow(ZtPredict[5][i],2);
				HtEG(1,1)=(robotX-ZtPredict[3][i])/pow(ZtPredict[5][i],2);
				if(i==0)
				{
					HtEG(0,3)=-HtEG(0,0);
					HtEG(0,4)=-HtEG(0,1);
					HtEG(1,3)=-HtEG(1,0);
					HtEG(1,4)=-HtEG(1,1);
					for(j=0;j<2;j++)
					{
						for(k=0;k<7;k++)
						{
							P(5+j,k)=P(k,5+j)=0;
						}
					}
				}
				else
				{
					HtEG(0,5)=-HtEG(0,0);
					HtEG(0,6)=-HtEG(0,1);
					HtEG(1,5)=-HtEG(1,0);
					HtEG(1,6)=-HtEG(1,1);
				}
				
				// Get Kalman Gain matrix from current reobserved landmark
				KtInnerTerm=(HtEG*P*HtEG.transpose())+QtEG;
				KtEG=P*HtEG.transpose()*(KtInnerTerm.inverse());
				
				//Innov=(ZtObsEGs-ZtPredictEGs);
				Innov=(ZtPredictEGs-ZtObsEGs);

				// Update state matrix
				X=X+(KtEG*Innov);
				if(i==0)
				{
					HtEG(0,3)=0;
					HtEG(0,4)=0;
					HtEG(1,3)=0;
					HtEG(1,4)=0;
				}
				
			}
			
			// Print Estimated position
			std::cout<<"Current position estimate is:("<<X(0,0)<<","<<X(1,0)<<")"<<endl;
			std::cout<<"Odometry estimate is:("<<robotX<<","<<robotY<<")"<<endl;

			// Store current position as history for next time step
			robotXHist=X(0,0);//robotX;
			robotYHist=X(1,0);//robotY;
			robotThHist=X(2,0);//robotTh;

			// Save data to text file
			ofstream xSlam("xSLAM.txt",ios::app);
			xSlam<<X(0,0)<<"	";
			xSlam.close();
			ofstream ySlam("ySLAM.txt",ios::app);
			ySlam<<X(1,0)<<"	";
			ySlam.close();
			ofstream xOdo("xOdo.txt",ios::app);
			xOdo<<robotX<<"	";
			xOdo.close();
			ofstream yOdo("yOdo.txt",ios::app);
			yOdo<<robotY<<"	";
			yOdo.close();
			ofstream ts("ts.txt",ios::app);
			ts<<robot.getVel()<<"	";
			ts.close();	
			//--------SLAM ends------------------------
			
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