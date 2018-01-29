/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3929.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

public class Robot extends IterativeRobot {
	
	//Motor controllers
	VictorSP leftFront;
	VictorSP leftRear;
	VictorSP rightFront;
	VictorSP rightRear;
	VictorSP pulley;
	VictorSP leftIntake;
	VictorSP rightIntake;
	
	//Controller groups
	SpeedControllerGroup leftMotors;
	SpeedControllerGroup rightMotors;
	
	//Drive
	DifferentialDrive drive;
	double leftPow;
	double rightPow;
	double drivePow;
	
	//Encoders
	Encoder leftEncoder;
	Encoder rightEncoder;
	double distance;
	
	//Gyros
	PIDTool pidGyro;
	SerialPort serial_port;
	com.kauailabs.navx.frc.AHRS imu;
	double angle;
	
	//Controllers
	Joystick driveStick;
	Joystick opStick;
		
	//Smart dashboard auto choices
	private static final String kLeftLeftSwAuto = "Left Start, Left Switch";
	private static final String kLeftRightSwAuto = "Left Start, Right Switch";
	private static final String kLeftLeftScAuto = "Left Start, Left Scale";
	private static final String kLeftRightScAuto = "Left Start, Right Scale";
	private static final String kRightLeftSwAuto = "Right Start, Left Switch";
	private static final String kRightRightSwAuto = "Right Start, Right Switch";
	private static final String kRightLeftScAuto = "Right Start, Left Scale";
	private static final String kRightRightScAuto = "Right Start, Right Scale";
	private static final String kCenterLeftSwAuto = "Center Start, Left Switch";
	private static final String kCenterRightSwAuto = "Center Start, Right Switch";
	private static final String kCenterLeftScAuto = "Center Start, Left Scale";
	private static final String kCenterRightScAuto = "Center Start, Right Scale";
	
	//Smart dashboard control choices
	private static final String kArcadeMode = "Arcade Driving Mode";
	private static final String kTankMode = "Tank Driving Mode =)";
	private static final String kConsoleMode = "Console Driving Mode";
	
	//Smart dashboard misc
	private String m_autoSelected;
	private SendableChooser<String> m_autoChooser = new SendableChooser<>();
	
	private String m_driveSelected;
	private SendableChooser<String> m_driveChooser = new SendableChooser<>();
	
	//Auton stages
	enum AutoStage{
		turnZero, driveOne, turnOne, driveTwo, turnTwo, driveThree, place
	}
	
	AutoStage stage;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		//leftEncoder.setDistancePerPulse(10); // wheel diameter * Math.PI);
		
		//Initialize motor controllers
		//Match
		leftFront = new VictorSP(0);
		leftRear = new VictorSP(3);
		rightFront = new VictorSP(1);
		rightRear = new VictorSP(2);
		pulley = new VictorSP(4);
		leftIntake = new VictorSP(5);
		rightIntake = new VictorSP(6);
		
		//Add motor controllers to control groups
		leftMotors = new SpeedControllerGroup(leftFront, leftRear);
		rightMotors = new SpeedControllerGroup(rightFront, rightRear);
		
		//Add controller groups to differential drive
		drive = new DifferentialDrive(leftMotors, rightMotors);
		
		//Initialize controllers
		driveStick = new Joystick(0);
		opStick = new Joystick(1);
		
		//Add auto choices to SmartDashboard
		m_autoChooser.addDefault("Left Start, Left Switch", kLeftLeftSwAuto);
		m_autoChooser.addObject("Left Start, Right Switch", kLeftRightSwAuto);
		m_autoChooser.addObject("Left Start, Left Scale", kLeftLeftScAuto);
		m_autoChooser.addObject("Left Start, Right Scale", kLeftRightScAuto);
		m_autoChooser.addObject("Right Start, Left Switch", kRightLeftSwAuto);
		m_autoChooser.addObject("Right Start, Right Switch", kRightRightSwAuto);
		m_autoChooser.addObject("Right Start, Left Scale", kRightLeftScAuto);
		m_autoChooser.addObject("Right Start, Right Scale", kRightRightScAuto);
		m_autoChooser.addObject("Center Start, Left Switch", kCenterLeftSwAuto);
		m_autoChooser.addObject("Center Start, Right Switch", kCenterRightSwAuto);
		m_autoChooser.addObject("Center Start, Left Scale", kCenterLeftScAuto);
		m_autoChooser.addObject("Center Start, Right Scale", kCenterRightScAuto);
		SmartDashboard.putData("Auto choices", m_autoChooser);
		
		//Add driving modes to SmartDashboard
		m_driveChooser.addDefault("Arcade Driving Mode", kArcadeMode);
		m_driveChooser.addObject("Tank Driving Mode", kTankMode);
		m_driveChooser.addObject("Console Driving Mode", kConsoleMode);
		SmartDashboard.putData("Drive choices", m_driveChooser);
		
		//Move robot auto to first stage
		stage = AutoStage.turnZero;
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	
	@Override
	public void autonomousInit() {
		m_autoSelected = m_autoChooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		distance = (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2;
		angle = imu.getFusedHeading();
		
		switch (m_autoSelected) {
			case kLeftLeftSwAuto:
				leftLeftSwAuto();
				break;
			case kLeftRightSwAuto:
				leftRightSwAuto();
				break;
			case kLeftLeftScAuto:
				leftLeftScAuto();
				break;
			case kLeftRightScAuto:
				leftRightScAuto();
				break;
			case kRightLeftSwAuto:
				rightLeftSwAuto();
				break;
			case kRightRightSwAuto:
				rightRightSwAuto();
				break;
			case kRightLeftScAuto:
				rightLeftScAuto();
				break;
			case kRightRightScAuto:
				rightRightScAuto();
				break;
			case kCenterLeftSwAuto:
				centerLeftSwAuto();
				break;
			case kCenterRightSwAuto:
				centerRightSwAuto();
				break;
			case kCenterLeftScAuto:
				centerLeftScAuto();
				break;
			case kCenterRightScAuto:
				centerRightScAuto();
				break;
			default:
				
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	
	public void teleopInit() {
		m_driveSelected = m_driveChooser.getSelected();
	}
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		if(driveStick.getRawButton(1)){
			leftEncoder.reset();
			rightEncoder.reset();
		} 
		
		if(driveStick.getRawButton(2)){
			imu.resetDisplacement();
		}
		
		//GTA

		switch (m_driveSelected) {
			//Arcade
			case kArcadeMode:
			drive.arcadeDrive(driveStick.getRawAxis(1), driveStick.getRawAxis(0), true);
			System.out.println("a");
			break;
			
			//Tank
			case kTankMode:
			drive.tankDrive(driveStick.getRawAxis(1) * -1, driveStick.getRawAxis(5) * -1, true);
			System.out.println("b");
			break;
			
			//GTA
			case kConsoleMode:
			drive.arcadeDrive(
			Math.pow((driveStick.getRawAxis(2) * (-1)) + driveStick.getRawAxis(3), 3),
			(driveStick.getRawAxis(0) * (1)), true);
				
			System.out.println("c");
			break;
			
			default:
			break;
		}
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
	}
	
	
	
	//METHODS
	
	/**
	 * Method to drive; reverses power to one side.
	 */
	public void drive(double powLeft, double powRight){
		drive.tankDrive(powLeft * -1, powRight * -1, true);
	}
	
	public void leftLeftSwAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(distance < 168) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < 90) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 50) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}
	}
	
	public void leftRightSwAuto() {
		switch(stage) {
		case turnZero:
			if(imu.getFusedHeading() < 90) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(distance < 168) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < -90) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 168) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(imu.getFusedHeading() < -90) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}	
	}
	
	public void leftLeftScAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(distance < 299) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < 90) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 30) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}
	}

	public void leftRightScAuto() {
		switch(stage) {
		case turnZero:
			if(imu.getFusedHeading() < 90) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(distance < 168) {
				drive(.5, .5);	
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < -90) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 168) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(imu.getFusedHeading() < -90) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(distance < 12) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}	
	}

	public void rightRightSwAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(distance < 168) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < -90) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 50) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}
	}
	
	public void rightLeftSwAuto() {
		switch(stage) {
		case turnZero:
			if(imu.getFusedHeading() < -90) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(distance < 168) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < 90) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 168) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(imu.getFusedHeading() < 90) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}	
	}
	
	public void rightRightScAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(distance < 299) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < -90) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 30) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}
	}

	public void rightLeftScAuto() {
		switch(stage) {
		case turnZero:
			if(imu.getFusedHeading() < -90) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(distance < 168) {
				drive(.5, .5);	
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < 90) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 168) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(imu.getFusedHeading() < 90) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(distance < 12) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}	
	}

	public void centerLeftSwAuto() {
		switch(stage) {
		case turnZero:
			if(imu.getFusedHeading() > - 45) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(distance < 238) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < 135) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 70) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}
	}
	
	public void centerRightSwAuto() {
		switch(stage) {
		case turnZero:
			if(imu.getFusedHeading() < 45) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(distance < 238) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < - 135) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 70) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
			break;
		case place:
			//do stuff
			break;
		default:
			break;
		}
	}
	
	public void centerLeftScAuto() {
		switch(stage) {
		case turnZero:
			if(imu.getFusedHeading() > - 45) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(distance < 238) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < 45) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 120) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(imu.getFusedHeading() < 90) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(distance < 70) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
		case place:
			//do stuff
			break;
		default:
			break;
		}
	}

	public void centerRightScAuto() {
		switch(stage) {
		case turnZero:
			if(imu.getFusedHeading() > 45) {
				drive(.3, -.3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(distance < 238) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(imu.getFusedHeading() < -45) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(distance < 120) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(imu.getFusedHeading() < -90) {
				drive(-.3, .3);
			}
			else {
				resetMeasures();
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(distance < 70) {
				drive(.5, .5);
			}
			else {
				resetMeasures();
				stage = AutoStage.place;
			}
		case place:
			//do stuff
			break;
		default:
			break;
		}
	}
	
	public void resetMeasures(){
		leftEncoder.reset();
		rightEncoder.reset();
		imu.resetDisplacement();
	}
}
