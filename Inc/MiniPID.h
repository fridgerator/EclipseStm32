#ifndef MINIPID_H
#define MINIPID_H

#include <stdint.h>
#include "math.h"

double clamp(double, double, double);

class MiniPID{
public:
	MiniPID(double, double, double);
	MiniPID(double, double, double, double);
	void setP(double);
	void setI(double);
	void setD(double);
	void setF(double);
	void setPID(double, double, double);
	void setPID(double, double, double, double);
	void setMaxIOutput(double);
	void setOutputLimits(double);
	void setOutputLimits(double,double);
	void setDirection(bool);
	void setSetpoint(double);
	void reset();
	void setOutputRampRate(double);
	void setSetpointRange(double);
	void setOutputFilter(double);
	double getOutput();
	double getOutput(double);
	double getOutput(double, double);
	uint8_t getMode();
	void setMode(uint8_t);
	void compute();
	double GetKp();
	double GetKi();
	double GetKd();
	void setTunings(double, double, double);
	double getMaxOutput();
	double getMinOutput();

	uint8_t AUTOMATIC=1;
	uint8_t MANUAL=0;

private:
	bool bounded(double, double, double);
	void checkSigns();
	void init();
	double P;
	double I;
	double D;
	double F;
	double error;
	double output;

	double maxIOutput;
	double maxError;
	double errorSum;

	double maxOutput; 
	double minOutput;

	double setpoint;

	double lastActual;

	bool firstRun;
	bool reversed;

	double outputRampRate;
	double lastOutput;

	double outputFilter;

	double setpointRange;

	uint8_t mode;
};
#endif
