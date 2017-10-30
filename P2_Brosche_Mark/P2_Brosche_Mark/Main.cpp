#include <iostream>
#include <fstream>
#include <cstdlib>
#include "Controller.h";

using namespace std;

const double deltaT = 0.01;
const double endTime = 10.0;

ControllerState *status;

void main()
{
	status = InitController();

	ofstream OutFile;
	OutFile.open("MagLev Controller Data.csv");
	OutFile << "t,i,x,x',x'',e(t),integral(e(t)),d(e(t))/dt";
	OutFile.close();

	for (double t = 0.0; t <= endTime; t += deltaT)
	{
		ComputeNS(status, deltaT);
		UpdateCS(status);
		PrintState(status, t);
	}
}
