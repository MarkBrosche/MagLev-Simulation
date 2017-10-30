#include "Controller.h";
#include "Euler.h";
#include <fstream>;
#include <iostream>;
#include <cstdlib>;

using namespace std;

//double initialPosition = 5.1;
// The constant k is defined as:
double k()
{
	return mass * gravity * x_0 * x_0 / (i_0 * i_0);
}
// Initialize the floating mass
ControllerState *InitController()
{
	ControllerState *state = new ControllerState;
	state->positionCS = 5.1;
	state->velocityCS = 0.0;
	state->errorCS = x_0 - state->positionCS;
	state->integralErrorCS = 0.0;
	state->derivativeErrorCS = 0.0;
	state->currentCS= i_0 - (state->errorCS * P + state->integralErrorCS * I + state->derivativeErrorCS * D);
	state->accelerationCS = (mass * gravity - state->currentCS * state->currentCS * k() / (state->positionCS * state->positionCS)) / mass;
	PrintState(state, 0.0);
	return(state);
}



// Compute the next state of the floating mass for its characteristics
void ComputeNS(ControllerState *state, double deltaT)
{
	state->errorNS = x_0 - state->positionCS;
	state->integralErrorNS = Euler(state->integralErrorCS,state->errorCS, deltaT);
	state->derivativeErrorNS = (state->errorNS - state->errorCS) / deltaT;
	state->velocityNS = Euler(state->velocityCS, state->accelerationCS, deltaT);
	state->positionNS = Euler(state->positionCS, state->velocityCS, deltaT);
	state->currentNS = i_0 - (state->errorNS * P + state->integralErrorNS * I + state->derivativeErrorNS * D);
	state->accelerationNS = (mass * gravity - state->currentNS * state->currentNS * k() / (state->positionNS * state->positionNS)) / mass;
}

// Update the current state of the floating mass 
void UpdateCS(ControllerState *state)
{
	state->errorCS = state->errorNS;
	state->integralErrorCS = state->integralErrorNS;
	state->derivativeErrorCS = state->derivativeErrorNS;
	state->positionCS = state->positionNS;
	state->velocityCS = state->velocityNS;
	state->accelerationCS = state->accelerationNS;
	state->currentCS = state->currentNS;
}

// Print out the current state of the floating mass characteristics
void PrintState(ControllerState *state, double t) 
{
	ofstream OutFile;
	OutFile.open("MagLev Controller Data.csv", std::ios_base::app);
	OutFile << '\n';
	OutFile << t << ',' << state->currentCS << ',' << state->positionCS
		<< ',' << state->velocityCS << ',' << state->accelerationCS
		<< ',' << state->errorCS << ',' << state->integralErrorCS
		<< ',' << state->derivativeErrorCS;
	OutFile.close();
}
