// Z-shaped function equation with closest distance to obstacle as variable
// a and b locate the extremes of the sloped portion of the curve
// mF= multiplying factor(return variable); iT= intermediate term used internally; bM= Max sensing range in front of the robot(speed dependent).
// trip= trip variable from AIS indicating the direction of motion of robot

double zFunc(ArRobot *thisRobot ,ArSick *thisLaser,double bM,int a=1000,int b=2500)
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