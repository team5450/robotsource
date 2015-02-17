/*
 * PID_straight.h
 *
 *  Created on: Jan 20, 2015
 *      Author: Brandon Trabucco
 */

#include "WPILib.h"

#ifndef SRC_PID_STRAIGHT_H_
#define SRC_PID_STRAIGHT_H_

// old vals: KP = 0.0045, KI = 0.2, KD = 0.0000375

class PIDControl
{
	double PID_correction = 0,
		PID_rate = 0,
		PID_area = 0,
		PID_ff = 0,
		PID_output = 0,
		KP,
		KI,
		KD,
		KF,
		deltat,
		output_cap;
	public:
		double PID_error[2] = {0, 0};
		PIDControl(double, double, double, double, double, double);
		double GetPID(double PID_sp, double PID_pv)
		{
			PID_error[0] = PID_sp - PID_pv;
			PID_rate = (PID_error[0] - PID_error[1]) / deltat;
			PID_area = PID_area + PID_error[0] * deltat;
			PID_ff = (PID_output)*KF;

			PID_correction = (PID_error[0] * KP) + (PID_area * KI) + (PID_rate * KD);
			PID_error[1] = PID_error[0];
			PID_output = PID_ff + PID_correction;

			if(PID_output > output_cap){ PID_output = output_cap; }
			else if(PID_output < -output_cap){ PID_output = -output_cap; }

			return PID_output;
		}
		void InitPID()
		{
			PID_error[0] = 0;
			PID_error[1] = 0;
			PID_area = 0;
			PID_output = 0;
		}
};

PIDControl::PIDControl(double p, double i, double d, double f, double time, double max)
{
	KP = p;
	KI = i;
	KD = d;
	KF = f;
	deltat = time;
	output_cap = max;
}

#endif /* SRC_PID_STRAIGHT_H_ */
