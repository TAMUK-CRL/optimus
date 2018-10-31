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
//   Variable control response(VCR) for adjusting AIS controlled velocity effectiveness.
//   lmvUc= nth state left motor velocity from AIS; rmvUc= nth state right motor velocity from AIS.
//   lmvCp= n-1 state left motor velocity from VCR; rmvCp= n-1 state right motor velocity from VCR.
//   mF= multiplying factor from zFunction with distance to closest obstacle as the variable.
//   cr[0]=delta for left motor; cr[1]= delta for right motor;
//

double *vcr(ArRobot *thisRobot,double cr[2],double lmvUc,double rmvUc,double mF, double lmvCp,double rmvCp)
{
	cr[0]=lmvCp+(mF*(lmvUc-lmvCp));
	cr[1]=rmvCp+(mF*(rmvUc-rmvCp));
	return cr;
}
