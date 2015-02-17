/*
 * speed_dampener.h
 *
 *  Created on: Jan 20, 2015
 *      Author: Brandon Trabucco
 */

#ifndef SRC_SPEEDCONTROL_H_
#define SRC_SPEEDCONTROL_H_

class SpeedControl
{
	double input[2] = {0, 0},
		output = 0,
		rate = 0,
		distance = 0,
		correction = 0,
		KD,
		deltat;
	public:
		SpeedControl(double, double);
		double GetSpeed(double joystick)
		{
			input[0] = joystick;
			distance = input[0] - input[1];
			rate = distance / deltat;
			correction = rate * KD;		//		d values can range from .005( full correction ) .0045(almost instantaneous)
			output = input[0] - correction;
			input[1] = output;

			return output;
		}
};

SpeedControl::SpeedControl(double d, double time)
{
	KD = d;
	deltat = time;
}

#endif /* SRC_SPEEDCONTROL_H_ */
