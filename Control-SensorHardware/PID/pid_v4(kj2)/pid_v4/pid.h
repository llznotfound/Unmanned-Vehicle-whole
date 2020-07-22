#ifndef PID_H_
#define PID_H_

class PID
{
private:
	float kp;
	float ki;
	float kd;
	float integrator;
	float previous_error;
	float frequency;

public:
	PID(void);
	~PID(void) {};

	// Sets the gains of the given PID
	void set_gains(float kp, float ki, float kd);

	// Process one step if the PID algorithm
	float process(float error);

	// Sets the PID frequency for gain compensation
	void set_frequency(float frequency);
};

#endif
