#include "WPILib.h"
#define INTAKE_WHEEL_SPEED 1.0
#define USE_DRIVE_TIMER 1

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public SampleRobot
{
//	RobotDrive myRobot; // robot drive system
	SendableChooser *chooser;

	Talon rightDriveMotor;
	Talon leftDriveMotor;
	Talon intakeWheel;
	Spark intakeTilt;
	Spark winchTal;
	Joystick driveLeftStick;
	Joystick driveRightStick;
	Joystick manipulatorStick;
	Timer toggleButtonTimer;
	Timer autoDriveTimer;
	Timer winchTimer;
	Timer shootTimer;
	//AnalogInput crateDistanceSensor;
	AnalogGyro driveGyro;
	DigitalInput shootLimitSwitch;
	AnalogInput wallDistanceSensor;
	DoubleSolenoid *hanger;
	Compressor *compressorPointer;
	int autoState;
	bool winchRetracted;
	bool shootTimerStarted;
	bool driveReverse;
	bool isGyroResetTelop;
	int turningButtonState;
	int turningButtonAngle;
	int currentTurningButton;

	const std::string autoNameDefault = "Default";
	const std::string autoNameLowBar= "Low Bar";
	const std::string autoNameMoatRampart = "Moat or Rampart";
	const std::string autoNameScore = "Low Bar Score (non functional)";

public:
	Robot() :
//			myRobot(0, 1),	// these must be initialized in the same order
			chooser(),
			rightDriveMotor(0),
			leftDriveMotor(1),
			intakeWheel(2),
			intakeTilt(3),
			winchTal(4),
			driveLeftStick(1),
			driveRightStick(0),
			manipulatorStick(2),
			toggleButtonTimer(),
			autoDriveTimer(),
			winchTimer(),
			shootTimer(),
			driveGyro(0),
			shootLimitSwitch(0),
			wallDistanceSensor(1)
	{
		autoState = 0;
		winchRetracted = false;
		isGyroResetTelop = false;
		hanger= new DoubleSolenoid(0, 3);
		driveReverse = false;
		toggleButtonTimer.Reset();
		toggleButtonTimer.Start();
		compressorPointer = new Compressor();
		compressorPointer->SetClosedLoopControl(true);

		shootTimerStarted = false;
		shootTimer.Reset();
		turningButtonState=0;
		turningButtonAngle=0;
		currentTurningButton=0;
		hanger->Set(DoubleSolenoid::kReverse);

		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		//myRobot.SetExpiration(0.1);
	}

	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameLowBar, (void*)&autoNameLowBar);
		chooser->AddObject(autoNameMoatRampart, (void*)&autoNameMoatRampart);
		chooser->AddObject(autoNameScore, (void*)&autoNameScore);
		SmartDashboard::PutData("Auto Modes", chooser);

		driveGyro.Reset();
		hanger->Set(DoubleSolenoid::kReverse);

		//CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		//CameraServer::GetInstance()->StartAutomaticCapture("cam0");
	}

	float smoothJoyStick(float joyInput)
	{
		return powf(joyInput,3);
	}

	//-1 is reverse and 1 is norm
	void tankDrive()
	{
		toggleDriveDirection();
		//float right = smoothJoyStick(driveRightStick.GetY());
		//float left = smoothJoyStick(driveLeftStick.GetY());
		float right = driveRightStick.GetY();
		float left = driveLeftStick.GetY();
//		leftDriveMotor.Set(left);
//		rightDriveMotor.Set(-right);
		if(!driveRightStick.GetTrigger())
		{
			if (driveReverse)
			{
				leftDriveMotor.Set(-right);
				rightDriveMotor.Set(left);
			}
			else
			{
				leftDriveMotor.Set(left);
				rightDriveMotor.Set(-right);
			}
			isGyroResetTelop = false;
		}
		else
		{
			if(isGyroResetTelop == false)
			{
				driveGyro.Reset();
				isGyroResetTelop = true;
			}
			if (driveReverse)
			{
				keepDriveStraight(driveRightStick.GetY(), driveRightStick.GetY(), 0);
			}
			else
			{
				keepDriveStraight(-driveRightStick.GetY(), -driveRightStick.GetY(), 0);
			}
		}
	}
/*
	void setDrive(float right, float left, bool reverse)
	{
		if (reverse)
		{
			leftDriveMotor.Set(-right);
			rightDriveMotor.Set(left);
		}
		else
		{
			leftDriveMotor.Set(left);
			rightDriveMotor.Set(-right);
		}
	}
*/
	void hangerPiston(int extend, int retract)
	{
		if (manipulatorStick.GetRawButton(extend))
		{
			hanger->Set(DoubleSolenoid::kForward);
		}
		else if (manipulatorStick.GetRawButton(retract))
		{
			hanger->Set(DoubleSolenoid::kReverse);
		}
	}
	void toggleDriveDirection()
	{
		if(driveLeftStick.GetRawButton(2))
		{
			driveReverse=true;
		}
		else
		{
			driveReverse=false;
		}
	}
	bool turnButtons()
	{
		int direction = 1;
		if (driveReverse==true)
		{
			direction=-1;
		}
		switch(turningButtonState)
		{
			case 0:
				turningButtonAngle=0;
				currentTurningButton=0;
				if (driveRightStick.GetRawButton(5))
				{
					driveGyro.Reset();
					turningButtonAngle=(-90*direction);
					currentTurningButton=5;
					turningButtonState++;
				}
				else if (driveRightStick.GetRawButton(4))
				{
					driveGyro.Reset();
					turningButtonAngle=(90*direction);
					currentTurningButton=4;
					turningButtonState++;
				}
				else if (driveRightStick.GetRawButton(2))
				{
					driveGyro.Reset();
					turningButtonAngle=(180*direction);
					currentTurningButton=2;
					turningButtonState++;
				}
				return true;
				break;
			case 1:
				if (!driveRightStick.GetRawButton(currentTurningButton) || turnGyro(turningButtonAngle))
				{
					turningButtonState++;
				}
				return false;
				break;
			case 2:
				if (!driveRightStick.GetRawButton(currentTurningButton))
				{
					turningButtonState=0;
				}
				return false;
				break;
			default:
				turningButtonState=0;
				break;
		}
	}
	double calculateWallDistance(bool averaged = true)
	{
		double rawVoltage;
		static double crateDistance;

		if(averaged)
			rawVoltage = (double)(wallDistanceSensor.GetAverageVoltage());
		else
			rawVoltage = (double)(wallDistanceSensor.GetVoltage());

		//MB1013
		//double VFiveMM = 0.0048359375;  //((4.952 / 5120) * 5);
		//crateDistance = ((rawVoltage * 5 * 0.0393) / VFiveMM);  //Units inch

		//MB1030
		double VFiveMM = 0.009671875;
		crateDistance = rawVoltage / VFiveMM;

		return crateDistance;
	}
	void intakeWheelControl(int mIntakeInButton, int mIntakeOutButton)
	{
		if (manipulatorStick.GetRawButton(mIntakeInButton) || driveLeftStick.GetTrigger())
		{
			intakeWheel.Set(-INTAKE_WHEEL_SPEED);
		}
		else if (manipulatorStick.GetRawButton(mIntakeOutButton))
		{
			intakeWheel.Set(INTAKE_WHEEL_SPEED);
		}
		else
		{
			intakeWheel.Set(0);
		}
	}

	void tiltControl(int up, int down)
	{
		/*if (manipulatorStick.GetRawButton(up))
		{
			intakeTilt.Set(0.3);
		}
		else if (manipulatorStick.GetRawButton(down))
		{
			intakeTilt.Set(-0.1);
		}
		else
		{
			intakeTilt.Set(0);
		}*/
		intakeTilt.Set(manipulatorStick.GetY());
	}

	void winchControl()
	{
		//pulls back winc
		if (manipulatorStick.GetRawButton(2) && !winchRetracted)
		{
			winchTimer.Start();

			if(winchTimer.Get() > 0.025)
			{
				if(!shootLimitSwitch.Get())
				{
					winchRetracted = true;
				}
			}
			winchTal.SetSpeed(0.5);
		}
		else
		{
			winchTimer.Reset();
			if(!manipulatorStick.GetRawButton(2))
			{
				winchRetracted = false;
			}
			winchTal.SetSpeed(0.0);
		}
	}

	void ShootBall()
	{
		/*
		if(!shootTimerStarted)
		{
			shootTimer.Start();
			shootTimerStarted = true;
		}
		else
		{
			if(shootTimer.Get() > 0.3)
			{
				//Release winch
				if(shootTimer.Get() <= 0.4)
					winchTal.SetSpeed(1.0);
				else
					winchTal.SetSpeed(0.0);
			}
		}
		*/
		winchTal.SetSpeed(-0.75);
	}

	void StopShooting()
	{
		//Stop winch
		if(!manipulatorStick.GetRawButton(2))
			winchTal.SetSpeed(0.0);

		shootTimer.Stop();
		shootTimer.Reset();
		shootTimerStarted = false;
	}

	void ShooterControl()
	{
		//Shoots the ball
		if (manipulatorStick.GetRawButton(4))
		{
			ShootBall();
		}
		else
		{
			StopShooting();
		}
	}

	void stopRobotDrive()
	{
		leftDriveMotor.Set(0.0);
		rightDriveMotor.Set(0.0);
	}

	void resetDrive(bool isTimerBased)
	{
		if(isTimerBased)
		{
			autoDriveTimer.Reset();
			autoDriveTimer.Start();
			driveGyro.Reset();
		}
		else
		{
			//leftDriveEncoder.Reset();
			//rightDriveEncoder.Reset();
		}
	}

	void keepDriveStraight(float leftDriveVel, float rightDriveVel, float targetAngle)
	{
		float error = 0, correctionFactor;
		error = targetAngle - driveGyro.GetAngle();
		correctionFactor = (error/75.0);

		if(targetAngle > (driveGyro.GetAngle() - 0.5) || targetAngle < (driveGyro.GetAngle() + 0.5))
		{
			leftDriveMotor.Set((-leftDriveVel) + correctionFactor);
			rightDriveMotor.Set(rightDriveVel + correctionFactor);
		}
		else
		{
			leftDriveMotor.Set(-leftDriveVel);
			rightDriveMotor.Set(rightDriveVel);
		}
	}

	bool turnGyro(float rAngle)
	{
		float error = 0;
		//Positive gyro angle means turning left
		if(rAngle < driveGyro.GetAngle())
		{
			error = fabs(rAngle) - driveGyro.GetAngle();
			if(driveGyro.GetAngle() <= fabs(rAngle) && fabs(error) > 2.0)
			{
				//turn left
				leftDriveMotor.Set((error/45) + 0.2); //0.8  div 140
				rightDriveMotor.Set((error/45) + 0.2); //0.8
			}
			else
			{
				stopRobotDrive();
				return true;
			}
		}
		else if(rAngle > driveGyro.GetAngle())
		{
			error = -rAngle - driveGyro.GetAngle();
			if(driveGyro.GetAngle() >= -rAngle && fabs(error) > 2.0)
			{
				//turn right
				leftDriveMotor.Set((error/45) - 0.2); //-0.8
				rightDriveMotor.Set((error/45) - 0.2); //-0.8
			}
			else
			{
				stopRobotDrive();
				return true;
			}
		}
		else
		{
			stopRobotDrive();
			return true;
		}

		return false;
	}

	//Prior to calling this function you must call resetDrive1
	bool autoDriveRobot(float velocityLeft, float velocityRight, float timeSec, float distanceInch, bool isTimerBased)
	{
		if(isTimerBased)
		{
			if(autoDriveTimer.Get() <= timeSec)
			{
				//leftDriveMotor.Set(-velocityLeft);
				//rightDriveMotor.Set(velocityRight);
				keepDriveStraight(velocityLeft, velocityRight, 0);
			}
			else
			{
				stopRobotDrive();
				return true;
			}
		}
		else
		{
			/*if(fabs(convertDriveTicksToInches(rightDriveEncoder.GetRaw())) < fabs(distanceInch))
			{
				leftDriveMotor.Set(-velocityLeft);
				rightDriveMotor.Set(velocityRight);
			}
			else
			{
				stopRobotDrive();
				return true;
			}*/
		}
		return false;
	}

	void driveStraightAutonomous()
	{
		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				intakeTilt.Set(-0.4);
				Wait(1.0);
				autoState++;
				break;
			case 1:
				intakeTilt.Set(0.0);
				resetDrive(USE_DRIVE_TIMER);
				intakeWheel.Set(-1);
				Wait(0.5);
				autoState++;
				break;
			case 2:
				intakeWheel.Set(0.0);
				if(autoDriveRobot(0.6, 0.6, 1.0, 0, USE_DRIVE_TIMER)) //1.0 sec
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;

			case 3:
				if(autoDriveRobot(0.85, 0.85, 2.25, 0, USE_DRIVE_TIMER))
					autoState++;
				break;

			case 4:
				stopRobotDrive();
				break;
			default:
				stopRobotDrive();
				break;
		}
	}
	void moatRampartAutonomous()
	{
		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				intakeTilt.Set(-0.4);
				Wait(1.0);
				autoState++;
				break;
			case 1:
				intakeTilt.Set(0.0);
				resetDrive(USE_DRIVE_TIMER);
				intakeWheel.Set(-1);
				Wait(0.5);
				autoState++;
				break;
			case 2:
				intakeWheel.Set(0.0);
				if(autoDriveRobot(0.6, 0.6, 1.0, 0, USE_DRIVE_TIMER)) //1.0 sec
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;

			case 3:
				if(autoDriveRobot(0.85, 0.85, 2.5, 0, USE_DRIVE_TIMER))
					autoState++;
				break;

			case 4:
				stopRobotDrive();
				break;
			default:
				stopRobotDrive();
				break;
		}
	}
	void lowBarScoreAutonomous()
	{
		float wallTurnDistance;
		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				intakeTilt.Set(-0.4);
				Wait(1.0);
				autoState++;
				break;
			case 1:
				intakeTilt.Set(0.0);
				resetDrive(USE_DRIVE_TIMER);
				intakeWheel.Set(-1);
				Wait(0.5);
				autoState++;
				break;
			case 2:
				intakeWheel.Set(0.0);
				if(autoDriveRobot(0.5, 0.5, 1.0, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;

			case 3:
				if(autoDriveRobot(0.5, 0.5, 2.5, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;
			case 4:
				if (calculateWallDistance()>wallTurnDistance){}
				break;
			case 999:
				stopRobotDrive();
				break;
			default:
				stopRobotDrive();
				break;
		}
	}

	void lowBarAutonomous()
	{
		switch(autoState)
		{
			case 0:
				resetDrive(USE_DRIVE_TIMER);
				intakeTilt.Set(-0.4);
				Wait(1.0);
				autoState++;
				break;
			case 1:
				intakeTilt.Set(0.0);
				resetDrive(USE_DRIVE_TIMER);
				intakeWheel.Set(-1);
				Wait(0.5);
				autoState++;
				break;
			case 2:
				intakeWheel.Set(0.0);
				if(autoDriveRobot(0.6, 0.6, 1.0, 0, USE_DRIVE_TIMER))
				{
					resetDrive(USE_DRIVE_TIMER);
					autoState++;
				}
				break;

			case 3:
				if(autoDriveRobot(0.6, 0.6, 2.5, 0, USE_DRIVE_TIMER))
					autoState++;
				break;

			case 4:
				stopRobotDrive();
				break;
			default:
				stopRobotDrive();
				break;
		}
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void Autonomous()
	{
		std::string autoSelected = *((std::string*)chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		//driveGyro.Reset();

		while (IsAutonomous() && IsEnabled())
		{
			if(autoSelected == autoNameLowBar){
				/*myRobot.SetSafetyEnabled(false);
				myRobot.Drive(-0.5, 1.0); 	// spin at half speed
				Wait(2.0); 				//    for 2 seconds
				myRobot.Drive(0.0, 0.0); 	// stop robot
				*/
				lowBarAutonomous();
			}
			else if (autoSelected == autoNameMoatRampart)
			{
				moatRampartAutonomous();
			}
			else if (autoSelected == autoNameScore)
			{
				lowBarScoreAutonomous();
			}
			else
			{
				//Default Auto goes here
	//			myRobot.SetSafetyEnabled(false);
	//			myRobot.Drive(0.0, 0.0); 	// stop robot
				driveStraightAutonomous();
			}
			updateDashboard();
			Wait(0.005);				// wait for a motor update time
		}

	}
	void updateDashboard()
	{
		SmartDashboard::PutNumber("Wall Distance: ", calculateWallDistance(false));
		SmartDashboard::PutNumber("Gyro Reading: ", driveGyro.GetAngle());
	}
	/**
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl()
	{
		//myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled())
		{
			//if (turnButtons())
			//{
				tankDrive();
			//}
			intakeWheelControl(5,7);
			winchControl();
			ShooterControl();
			tiltControl(3, 2);  //Using stick now..not buttons
			updateDashboard();
			hangerPiston(6,8);
			Wait(0.005);				// wait for a motor update time
		}
	}

	/**
	 * Runs during test mode
	 */
	void Test()
	{
		while(IsTest())
		{
			rightDriveMotor.Set(0.0);
			leftDriveMotor.Set(0.0);
			intakeWheel.Set(0.0);
			intakeTilt.Set(0.0);
			winchTal.Set(0.0);
			Wait(0.005);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(Robot)
