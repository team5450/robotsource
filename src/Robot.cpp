#include "WPILib.h"
#include "SpeedControl.h"
#include "PIDControl.h"
#include <cmath>

class Robot: public IterativeRobot
{
	private:

	double kUpdatePeriod = 0.005, targetPosition = 0, conveyorRate[5] = {0, 0, 0, 0, 0};  // wheel radius in inches
	int p[4] = {0, 0, 0, 0};
	LiveWindow *lw;

	Joystick *a_stick;	// initialize joystick set to port 0 in FRC driver station
	Joystick *b_stick;  //

	Talon *r_wheel;			// initialize motor on port 0
	Talon *l_wheel;
	Talon *conveyor;
	Talon *intake;

	Solenoid *intake_arm_r;
	Solenoid *intake_arm_l;
	Solenoid *balance_arm;

	Compressor *c;

	Ultrasonic *s1;

	Encoder *l_wheel_encod;
	Encoder *r_wheel_encod;
	Encoder *conveyor_encod;

	PIDControl *r_wheel_PID;
	PIDControl *l_wheel_PID;
	PIDControl *conveyor_PID;
	PIDControl *conveyor_auto_PID;
	PIDControl *conveyor_rate_PID;
								//		KP, KI, KD, KF, deltat

	PIDController *test;

	SpeedControl *l_wheel_ctrl;
	SpeedControl *r_wheel_ctrl;
	// 				KD, deltat

	void RobotInit()
	{

		lw = LiveWindow::GetInstance();
		a_stick = new Joystick(0);	// initialize joystick set to port 0 in FRC driver station
		b_stick = new Joystick(1);  // joystick 2

		r_wheel = new Talon(0);			// initialize motor on port 0
		l_wheel = new Talon(1);
		conveyor = new Talon(2);
		intake = new Talon(3);

		intake_arm_r = new Solenoid(0);
		intake_arm_l = new Solenoid(1);
		balance_arm = new Solenoid(2);

		c = new Compressor(0);

		s1 = new Ultrasonic(0, 1);

		l_wheel_encod = new Encoder(8, 9, false, Encoder::EncodingType::k4X);
		r_wheel_encod = new Encoder(2, 3, false, Encoder::EncodingType::k4X);
		conveyor_encod = new Encoder(4, 5, false, Encoder::EncodingType::k4X);

		r_wheel_PID = new PIDControl(.006, 0, 0, 0, kUpdatePeriod, .35);
		l_wheel_PID = new PIDControl(.01, 0, 0, 0, kUpdatePeriod, .5);
		conveyor_PID = new PIDControl(.006, 0, 0, 0, kUpdatePeriod, 1);
		conveyor_auto_PID = new PIDControl(.004, 0, 0, 0, kUpdatePeriod, 400);
		conveyor_rate_PID = new PIDControl(.00004, 0, 0, 1, kUpdatePeriod, 1);
										//		KP, KI, KD, KF, deltat

		l_wheel_ctrl = new SpeedControl(.00490, kUpdatePeriod);
		r_wheel_ctrl = new SpeedControl(.00490, kUpdatePeriod);
										// 				KD, deltat
		c->SetClosedLoopControl(true);

		l_wheel->Set(0);
		r_wheel->Set(0);

		r_wheel_encod->Reset();
		l_wheel_encod->Reset();
	}

	void RobotDrive(double distance)
	{
		double wheel_diameter = 6,
				degrees_per_period = 360,
				wheel_circumference,
				wheel_turns,
				degrees_to_move;
		wheel_circumference = wheel_diameter*3.1415;		// calc wheel circumference
		wheel_turns = (distance*12)/wheel_circumference;	// calc number of wheel rotations based on ^ and distance to go in inches
		degrees_to_move = wheel_turns*degrees_per_period;	// calc this same value in encoder clicks
		SmartDashboard::PutNumber("Degrees", degrees_to_move);
		r_wheel_PID->InitPID();									// initialize PID
		l_wheel_PID->InitPID();
		r_wheel_encod->Reset();
		l_wheel_encod->Reset();
		l_wheel->Set((distance/abs(distance))*.25);
		do
		{
			r_wheel->Set(-1*r_wheel_PID->GetPID(degrees_to_move, r_wheel_encod->Get()));
			l_wheel->Set(l_wheel_PID->GetPID(r_wheel_encod->Get(), l_wheel_encod->Get())+(distance/abs(distance))*.25);
			SmartDashboard::PutNumber("Current Error r", r_wheel_PID->PID_error[0]);
			SmartDashboard::PutNumber("Current Error l", l_wheel_PID->PID_error[0]);
			SmartDashboard::PutNumber("Right Enc", r_wheel_encod->Get());
			SmartDashboard::PutNumber("Left Enc", l_wheel_encod->Get());
			Wait(kUpdatePeriod);
		}
		while((r_wheel_PID->PID_error[0] > 50 || r_wheel_PID->PID_error[0] < -50) || (l_wheel_PID->PID_error[0] > 50 || l_wheel_PID->PID_error[0] < -50));
		r_wheel->Set(0);
		l_wheel->Set(0);
		SmartDashboard::PutNumber("Current Error r", 10102);
		SmartDashboard::PutNumber("Current Error l", 10102);
		Wait(1);

	}

	void RobotTurn(double degrees)
	{
		double robot_diameter = 37.25,
				wheel_diameter = 6,
				degrees_per_period = 360,
				robot_circumference,
				wheel_circumference,
				wheel_turns,
				degree_ratio,
				turn_distance,
				degrees_to_turn;
		robot_circumference = robot_diameter*3.1415;		// calc robot circumference
		wheel_circumference = wheel_diameter*3.1415;		// calc wheel circumference
		degree_ratio = degrees/360;							// calc degrees desired out of 360 to get a ratio
		turn_distance = robot_circumference*degree_ratio;	// calc the portion of circumference to turn based on ^
		wheel_turns = turn_distance/wheel_circumference;	// calc this same distance in units of wheel turns
		degrees_to_turn = degrees_per_period*wheel_turns;	// calc this same value in encoder clicks
		SmartDashboard::PutNumber("Degrees", degrees_to_turn);
		l_wheel_PID->InitPID();								// initialize PID
		r_wheel_PID->InitPID();
		r_wheel_encod->Reset();
		l_wheel_encod->Reset();
		l_wheel->Set((degrees/abs(degrees))*.25);
		do
		{
			r_wheel->Set(-1*r_wheel_PID->GetPID(-1*degrees_to_turn, r_wheel_encod->Get()));
			l_wheel->Set(l_wheel_PID->GetPID(-1*r_wheel_encod->Get(), l_wheel_encod->Get())+(degrees/abs(degrees))*.25);
			SmartDashboard::PutNumber("Current Error r", r_wheel_PID->PID_error[0]);
			SmartDashboard::PutNumber("Current Error l", l_wheel_PID->PID_error[0]);
			SmartDashboard::PutNumber("Right Enc", r_wheel_encod->Get());
			SmartDashboard::PutNumber("Left Enc", l_wheel_encod->Get());
			Wait(kUpdatePeriod);
		}
		while((r_wheel_PID->PID_error[0] > 50 || r_wheel_PID->PID_error[0] < -50) || (l_wheel_PID->PID_error[0] > 50 || l_wheel_PID->PID_error[0] < -50));
		l_wheel->Set(0);
		r_wheel->Set(0);
		Wait(1);
	}

	void RobotLift(double level)
	{
		double degrees_per_level = 800,
				degrees_to_level;
		degrees_to_level = level*degrees_per_level;
		conveyor_rate_PID->InitPID();
		conveyor_auto_PID->InitPID();							// initialize PID
		conveyorRate[3] = 0;
		conveyorRate[2] = 0;
		conveyorRate[1] = 0;
		conveyorRate[0] = 0;
		conveyor_encod->Reset();
		while((conveyor_auto_PID->PID_error[0] > 50) || (conveyor_auto_PID->PID_error[0] < -50))
		{
			MoveConveyor(conveyor_auto_PID->GetPID(degrees_to_level, conveyor_encod->Get()));
			Wait(kUpdatePeriod);
		}
		conveyor->Set(0);
		Wait(1);
	}

	void MoveConveyor(double desiredRate)
	{
		// update current rate
		conveyorRate[3] = conveyorRate[2];
		conveyorRate[2] = conveyorRate[1];
		conveyorRate[1] = conveyorRate[0];
		conveyorRate[0] = conveyor_encod->GetRate();
		SmartDashboard::PutNumber("Conveyor Rate", conveyorRate[0]);

		// pass filter
		conveyorRate[4] = (conveyorRate[0] + conveyorRate[1] + conveyorRate[2] + conveyorRate[3])/4;

		// get PID
		conveyor->Set(conveyor_rate_PID->GetPID(desiredRate, conveyorRate[4]));
		SmartDashboard::PutNumber("Conveyor Error", conveyor_rate_PID->PID_error[0]);
	}

	void AutonomousInit()
	{
		RobotDrive(5);
		RobotDrive(-5);
		RobotTurn(90);
		RobotTurn(-90);
		RobotLift(1);
	}

	void AutonomousPeriodic()
	{
			// runs every 2ms
	}

	void TeleopInit()
	{
			// runs at the start of Teleop period
	}

	void TeleopPeriodic()
	{
		while(true)
		{
			//PID wheel
			SmartDashboard::PutNumber("Right Enc", r_wheel_encod->Get());
			SmartDashboard::PutNumber("Left Enc", l_wheel_encod->Get());
			SmartDashboard::PutNumber("Conveyor Enc", conveyor_encod->Get());

			if(a_stick->GetRawAxis(5) > .1 || a_stick->GetRawAxis(5) < -.1)
			{
				r_wheel->Set(r_wheel_ctrl->GetSpeed(a_stick->GetRawAxis(5)/1.2));
				} else if(b_stick->GetRawAxis(5) > .1 || b_stick->GetRawAxis(5) < -.1) {
				r_wheel->Set(r_wheel_ctrl->GetSpeed(b_stick->GetRawAxis(5)/1.2));
				} else {
				r_wheel->Set(r_wheel_ctrl->GetSpeed(0));
			}

			// right wheel

			if(a_stick->GetRawAxis(1) > .1 || a_stick->GetRawAxis(1) < -.1)
			{
				l_wheel->Set(l_wheel_ctrl->GetSpeed(-1*a_stick->GetRawAxis(1)/1));
				} else if(b_stick->GetRawAxis(1) > .1 || b_stick->GetRawAxis(1) < -.1) {
				l_wheel->Set(l_wheel_ctrl->GetSpeed(-1*b_stick->GetRawAxis(1)/1));
				} else {
				l_wheel->Set(l_wheel_ctrl->GetSpeed(0));
			}

			// This is the misc. section --,
			//							   V

			// Elevator

			if((a_stick->GetPOV() == 0) && p[3] > 0)
			{
				conveyor_rate_PID->InitPID();
				conveyorRate[3] = 0;
				conveyorRate[2] = 0;
				conveyorRate[1] = 0;
				conveyorRate[0] = 0;
				p[3] = 0;
				} else if((b_stick->GetPOV() == 0) && p[3] > 0){
				conveyor_rate_PID->InitPID();
				conveyorRate[3] = 0;
				conveyorRate[2] = 0;
				conveyorRate[1] = 0;
				conveyorRate[0] = 0;
				p[3] = 0;
				} else if((a_stick->GetPOV() == 180) && p[3] > 0){
				conveyor_rate_PID->InitPID();
				conveyorRate[3] = 0;
				conveyorRate[2] = 0;
				conveyorRate[1] = 0;
				conveyorRate[0] = 0;
				p[3] = 0;
				} else if((b_stick->GetPOV() == 180) && p[3] > 0){
				conveyor_rate_PID->InitPID();
				conveyorRate[3] = 0;
				conveyorRate[2] = 0;
				conveyorRate[1] = 0;
				conveyorRate[0] = 0;
				p[3] = 0;
				} else if((a_stick->GetPOV() == 0) && p[3] == 0){
											// get rate PID
				MoveConveyor(400);
				} else if((b_stick->GetPOV() == 0) && p[3] == 0){
											// get rate PID
				MoveConveyor(400);
				} else if((a_stick->GetPOV() == 180) && p[3] == 0){
											// get rate PID
				MoveConveyor(-200);
				} else if((b_stick->GetPOV() == 180) && p[3] == 0){
											// get rate PID
				MoveConveyor(-200);
				} else if(p[3] == 0){
											// set pos and init PID
				conveyor_encod->Reset();
				conveyor_PID->InitPID();
				p[3]++;
				} else {
											// get PID
				conveyor->Set(conveyor_PID->GetPID(0, conveyor_encod->Get()));
				p[3]++;
			}

			// Intake/Output Wheels

			if(a_stick->GetRawAxis(2) > .05)
			{
				intake->Set(a_stick->GetRawAxis(2));
				} else if(b_stick->GetRawAxis(2) > .05) {
				intake->Set(b_stick->GetRawAxis(2));
				} else if(a_stick->GetRawAxis(3) > .05) {
				intake->Set(-1*(a_stick->GetRawAxis(3)));
				} else if(b_stick->GetRawAxis(3) > .05) {
				intake->Set(-1*(b_stick->GetRawAxis(3)));
				} else {
				intake->Set(0);
			}

			// Solenoid initiators

			if(a_stick->GetRawButton(6))
			{
				intake_arm_r->Set(false);
				} else if(b_stick->GetRawButton(6)){
				intake_arm_r->Set(false);
				} else {
				intake_arm_r->Set(true);
			}

			if(a_stick->GetRawButton(5))
			{
				intake_arm_l->Set(false);
				} else if(b_stick->GetRawButton(5)){
				intake_arm_l->Set(false);
				} else {
				intake_arm_l->Set(true);
			}

			Wait(kUpdatePeriod);
		}
	}

	void TestPeriodic()
	{
		while(true)
		{
			lw->Run();
			Wait(kUpdatePeriod);
		}
	}
};

START_ROBOT_CLASS(Robot);		// run the robot class (start robot)
