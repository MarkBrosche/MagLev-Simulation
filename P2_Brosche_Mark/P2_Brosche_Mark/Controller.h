#pragma once
#define P 5
#define I 0.5
#define D 0.2
#define gravity 9.80665
#define mass 1
#define x_0 5
#define i_0 1

//Structure to define the variables associated with the Maglev Controller
struct ControllerState
{
	double positionCS;
	double positionNS;
	double velocityCS;
	double velocityNS;
	double accelerationCS;
	double accelerationNS;
	double errorCS;
	double errorNS;
	double integralErrorCS;
	double integralErrorNS;
	double derivativeErrorCS;
	double derivativeErrorNS;
	double currentCS;
	double currentNS;
};


ControllerState *InitController();
double k();
void ComputeNS(ControllerState *state, double deltaT);
void UpdateCS(ControllerState *state);
void PrintState(ControllerState *state, double t);
