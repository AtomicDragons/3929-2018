/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3929.robot;

import java.rmi.UnexpectedException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
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
	
	double kPgyro = 0.05;
	final double kIgyro = 0.0;
	final double kDgyro = 0.0;
	final double MAX_ROTATION_INPUT = .6;
	final double MINM_ROTATION_INPUT = .4;
	final double MAX_VISION_INPUT = 0.5;
	
	PIDTool pidGyro;
	
	//Motor controllers
	WPI_VictorSPX leftFront;
	WPI_TalonSRX leftRear;
	WPI_TalonSRX rightFront;
	WPI_VictorSPX rightRear;
	WPI_TalonSRX elevator;
	
	Spark leftIntake;
	Spark rightIntake;	
	Spark frontHanger;
	VictorSP backHanger; 
	
	//Solenoids
	Compressor compressor;
	DoubleSolenoid clawSully;
	DoubleSolenoid liftSully;
	boolean isLifted;
	boolean isOpened;
	
	//Sensors
	DigitalInput elevatorLimitSwitchTop;
	DigitalInput elevatorLimitSwitchBottom;
	
	//Controller values
	private double error;
	private double signal;
	
	VictorSP light;
	
	//Controller groups
	SpeedControllerGroup leftMotors;
	SpeedControllerGroup rightMotors;
	
	double throttleLeft;
	double throttleRight;
	
	//Encoders
	Encoder leftEncoder; 
	Encoder rightEncoder; 
	
	//Drive
	DifferentialDrive drive;
	double leftPow;
	double rightPow;
	double drivePow;
	Timer timer;

	//Constants
	final double ROBOT_HALF = 14; //Half the length of the robot
	final double ROBOT_LENGTH = 28; //The length of the robot
	final double PULSE_CONVERSION =  .27692308; //inches to encoder
	final double TIME_CONVERSION = 1000;
	//final double kP;
	//final double kI;
	//final double kD;
	
	final double elevatorScale = 0;
	final double elevatorSwitch = 0; 

	
	//Gyros
	AHRS imu;
	double angle;
	double dAngle;
	
	//Controllers
	Joystick driveStick;
	Joystick opStick;
		
	//Smart dashboard auto choices
	private static final String kLeftStart = "Left Start";
	private static final String kRightStart = "Right Start";
	private static final String kCenterStart = "Center Start";

	String gameData;
	private double autoDelay = 0;
	
	Preferences prefs;
	
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
	enum AutoMode {
		kLeftLeftSwAuto, kLeftRightSwAuto, kLeftLeftScAuto, kLeftRightScAuto, 
		kCenterRightSwAuto, kCenterLeftSwAuto, kCenterRightScAuto, kCenterLeftScAuto,
		kRightLeftSwAuto, kRightRightSwAuto, kRightLeftScAuto, kRightRightScAuto, kDummy,
	}
	enum AutoStage{
		turnZero, driveOne, turnOne, driveTwo, turnTwo, driveThree, turnThree, driveFour, place, end
	}
	private boolean elevatorZeroed;
	AutoStage stage;
	AutoMode mode;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		//autodelay config
		prefs = Preferences.getInstance();
		autoDelay = prefs.getDouble("Auto Delay", 0);
		
		pidGyro = new PIDTool(kPgyro, kIgyro, kDgyro, 0, -MAX_ROTATION_INPUT, MAX_ROTATION_INPUT);
		
		
		//CameraServer.getInstance().startAutomaticCapture();
		//CameraServer.getInstance().startAutomaticCapture();
		
		imu = new AHRS(SPI.Port.kMXP);
		
		RobotSchematic map = new RobotSchematic();
		
		//leftEncoder.setDistancePerPulse(10); // wheel diameter * Math.PI);
		
		//Initialize motor controllers
		leftFront = new WPI_VictorSPX(map.LEFT_FRONT_MOTOR_PORT);
		leftRear = new WPI_TalonSRX(map.LEFT_REAR_MOTOR_PORT);//0
		leftFront.follow(leftRear);
		rightRear = new WPI_VictorSPX(map.RIGHT_REAR_MOTOR_PORT);
		rightFront = new WPI_TalonSRX(map.RIGHT_FRONT_MOTOR_PORT);
		rightRear.follow(rightFront);
		
		elevator = new WPI_TalonSRX(map.ELEVATOR_MOTOR_PORT);
		frontHanger = new Spark(map.FRONT_HANGER_MOTOR_PORT); 
		backHanger = new VictorSP(map.REAR_HANGER_MOTOR_PORT);
		
		leftEncoder = new Encoder(1,0,false,Encoder.EncodingType.k4X);
		
		rightEncoder = new Encoder(3,2,false,Encoder.EncodingType.k4X);
		
		isOpened = false;
		isLifted = false;
		  
		leftIntake = new Spark(map.LEFT_INTAKE_MOTOR_PORT);
		rightIntake = new Spark(map.RIGHT_INTAKE_MOTOR_PORT);
		
		
		//Initialize solenoids
		compressor = new Compressor();
		clawSully = new DoubleSolenoid(map.RIGHT_CLAW_SULLY_PORT, map.LEFT_CLAW_SULLY_PORT);
		liftSully = new DoubleSolenoid(map.RIGHT_LIFT_SULLY_PORT, map.LEFT_LIFT_SULLY_PORT);
		
		//Initialize/configures sensors
		elevatorLimitSwitchTop = new DigitalInput(map.LIMIT_SWITCH_TOP_PORT);
		elevatorLimitSwitchBottom = new DigitalInput(map.LIMIT_SWITCH_BOTTOM_PORT);
		
		//leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		//rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
		elevatorZeroed = false;
		
		//Add motor controllers to control groups
		leftMotors = new SpeedControllerGroup(leftFront);
		rightMotors = new SpeedControllerGroup(rightFront);
		throttleLeft = 0;
		throttleRight = 0;
		
		//Add controller groups to differential drive
		drive = new DifferentialDrive(leftMotors, rightMotors);
		
		//Initialize controllers
		driveStick = new Joystick(0);
		opStick = new Joystick(1);
		
		//Add data to SmartDashboard
		signal = 0.0;
		error = 0.0;
		
		SmartDashboard.putNumber("Signal", signal);
		SmartDashboard.putNumber("Angle", imu.getAngle());
		SmartDashboard.putNumber("Distance", leftEncoder.get() * PULSE_CONVERSION);
		SmartDashboard.putNumber("Error", error);
		
		//Add auto choices to SmartDashboard
		m_autoChooser.addDefault("Left Start", kLeftStart);
		m_autoChooser.addObject("Right Start", kRightStart);
		m_autoChooser.addObject("Center Start", kCenterStart);

		SmartDashboard.putData("Auto choices", m_autoChooser);
		
		//Add driving modes to SmartDashboard
		m_driveChooser.addDefault("Tank Driving Mode", kTankMode);
		m_driveChooser.addObject("Arcade Driving Mode", kArcadeMode);
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
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		m_autoSelected = m_autoChooser.getSelected();
		stage = AutoStage.turnZero;
		elevatorZeroed = false;
		resetEncoders();
		resetGyro();
		
		 if(m_autoSelected == kLeftStart) {
			if(gameData.substring(0,1).equals("R")) {
				mode = AutoMode.kLeftRightSwAuto;
			}
			if(gameData.substring(0,1).equals("L")) {
				mode = AutoMode.kLeftLeftSwAuto;
			}
			if(gameData.substring(1,2).equals("R")) {
				mode = AutoMode.kLeftRightScAuto;
			}
			if(gameData.substring(1,2).equals("L")) {
				mode = AutoMode.kLeftLeftScAuto;
			}
		}
		else if(m_autoSelected == kCenterStart) {
			if(gameData.substring(0,1).equals("R")) {
				mode = AutoMode.kCenterRightSwAuto;
			}
			if(gameData.substring(0,1).equals("L")) {
				mode = AutoMode.kCenterLeftSwAuto;
			}
			if(gameData.substring(1,2).equals("R")) {
				mode = AutoMode.kCenterRightScAuto;
			}
			if(gameData.substring(1,2).equals("L")) {
				mode = AutoMode.kCenterLeftScAuto;
			}
		}
		else if(m_autoSelected == kRightStart) {
			if(gameData.substring(0,1).equals("R")) {
				mode = AutoMode.kRightRightSwAuto;
			}
			if(gameData.substring(0,1).equals("L")) {
				mode = AutoMode.kRightLeftSwAuto;
			}
			if(gameData.substring(1,2).equals("R")) {
				mode = AutoMode.kRightRightScAuto;
			}
			if(gameData.substring(1,2).equals("L")) {
				mode = AutoMode.kRightLeftScAuto;
			}
		}
		else {
			mode = AutoMode.kDummy;
		}
		
		mode = AutoMode.kRightRightScAuto;
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		dAngle = imu.getFusedHeading() - angle;
		angle = imu.getFusedHeading();
		SmartDashboard.putNumber("Angle", imu.getAngle());
		SmartDashboard.putNumber("Distance", leftEncoder.get() * PULSE_CONVERSION);
		SmartDashboard.putNumber("Error", error);
		if(!elevatorZeroed) {
			if(mode == AutoMode.kLeftRightScAuto || mode == AutoMode.kLeftLeftScAuto ||
			   mode == AutoMode.kCenterRightScAuto || mode == AutoMode.kCenterRightScAuto ||
			   mode == AutoMode.kRightRightScAuto || mode == AutoMode.kRightLeftScAuto) {
				elevatorZero();
			}
		}
		switch (mode) {
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
			case kDummy:
				
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
		compressor.setClosedLoopControl(true); 
		//GTA
		//signal = ( leftFront.getMotorOutputPercent() + rightFront.getMotorOutputPercent() ) / 2;
		switch (m_driveSelected) {
			
			//Arcade
			case kArcadeMode:
				//this hasn't been tested!!!!!!
				/*if(Math.abs(throttleLeft + (driveStick.getRawAxis(1)) / 3) < 1 ) {
					throttleLeft += (driveStick.getRawAxis(1)) / 10;
				}
				if(Math.abs(throttleRight + (driveStick.getRawAxis(0)) / 3) < 1) {
					throttleRight += (driveStick.getRawAxis(0)) / 10;
				}
				System.out.println(throttleLeft + " ||| " + throttleRight);
				drive.arcadeDrive(throttleLeft, throttleRight, true);*/
				
				drive.arcadeDrive(driveStick.getRawAxis(1), driveStick.getRawAxis(0), true);
			break;
			
			//Tank
			case kTankMode:

			drive.tankDrive(driveStick.getRawAxis(1) * - 1, driveStick.getRawAxis(5) * - 1, true);
			break;
			
			//GTA
			case kConsoleMode:
			drive.arcadeDrive(
			Math.pow((driveStick.getRawAxis(2) * (-1)) + driveStick.getRawAxis(5), 3),
			Math.pow((driveStick.getRawAxis(0) * (1)), 3), true);
			break;
			
			default:
			break;
		}
		
		//OpStick outtake
		if(opStick.getRawButton(2)) {
			leftIntake.set(1.0);
			rightIntake.set(-1.0);
		}
		else if(opStick.getRawButton(1)) {
			leftIntake.set(-1.0);
			rightIntake.set(1.0);
		}
		//DriveStick intake
		else if(driveStick.getRawButton(5)){
			leftIntake.set(-1.0); 
			rightIntake.set(1.0);
		}
		//DriveStick outtake
		else if(driveStick.getRawButton(6)){
			leftIntake.set(1.0);
			rightIntake.set(-1.0);
		}
		else {
			leftIntake.set(0);
			rightIntake.set(0);
		}
		/*
		//DriveStick climber
		if(driveStick.getRawButton(1)) {
			frontHanger.set(.6);
			backHanger.set(-.6);
		}
		else {
			frontHanger.set(0);
			backHanger.set(0);
		}
		*/
		//Opstick elevator controls
		if(Math.abs(opStick.getRawAxis(1)) > .3) {
			
			if( (elevatorLimitSwitchTop.get() && opStick.getRawAxis(1) < 0) ||
				(elevatorLimitSwitchBottom.get() && opStick.getRawAxis(1) > 0)	) {
				elevator.set(-.15);
			}
			else {
				if(opStick.getRawAxis(1) - .15 < -1) {
					elevator.set(-1);
				}
				else {
					elevator.set(opStick.getRawAxis(1) - .15);
				}
			}
		}
		else {
			elevator.set(-.15);
		}
		
		///Raise claws
		if(opStick.getRawButton(11)) {
			liftSully.set(DoubleSolenoid.Value.kReverse);
		}
		//Lower claws
		else{
			liftSully.set(DoubleSolenoid.Value.kForward);
		}
	
		
		//Open claws
		if(opStick.getRawButton(7)) {
			clawSully.set(DoubleSolenoid.Value.kForward);
		}
		//Close claws
		else {
			clawSully.set(DoubleSolenoid.Value.kReverse);
		}
		//Opstick climber 
		
		if(opStick.getRawButton(6)) {
			frontHanger.set(-.6);
			backHanger.set(.6);
		}
		
		/*if(opStick.getRawButton(4) && driveStick.getRawButton(6)) { 
			//the second button is supposed to be "back" on driver controller - test
			frontHanger.set(.6);
			backHanger.set(-.6);
		}
		*/
		else {
			frontHanger.set(0);
			backHanger.set(0);
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
	 * Method to zero the elevator and returns whether the elevator is zeroed or not.
	 * @return True if the elevator is not zeroed, false otherwise.
	 */
	public void elevatorZero() {
		if(!elevatorLimitSwitchTop.get()) {
			elevator.set(-.8);		
		}
		else {
			elevator.set(-.15);
			elevatorZeroed = true;
		}
	}
	
	/**
	 * Simple method to drive a certain distance using proportion
	 * @param target Desired distance.
	 * @return True if the robot has arrived at the target, false otherwise.
	 */
	public boolean driveTo(double target) {
		if(target < 24) {
			error = target - (leftEncoder.get() * PULSE_CONVERSION);
			if(error <= 0) {
				drive.tankDrive(0.0, 0.0);
				timer.delay(.2);
				resetGyro();
				resetEncoders();
				return true;
			}
			else {
				drive.arcadeDrive(.5, imu.getAngle() * - .2);
			}
			return false;
		}
		else {
			error = target - (leftEncoder.get() * PULSE_CONVERSION);
			if(error < target * .05) {
				drive.tankDrive(0.0, 0.0);
				timer.delay(.2);
				resetGyro();
				resetEncoders();
				return true;
			}
			if( (error / target) * 1.8 > .5) {
				drive.arcadeDrive( (error / target) * 1.8, imu.getAngle() * - .2);
			}
			else {
				drive.arcadeDrive(.5, imu.getAngle() * - .2);
			}
			return false;
		}
	}
	
	/**
	 * Method to drive a certain distance using PID.
	 * @param target Desired distance.
	 * @return True if the robot has arrived at the target, false otherwise.
	 */
	public boolean driveToPID(double target) {
		target = target * PULSE_CONVERSION;
		//remember to set coefficients kP, kI, & kD
		error = ( leftFront.getClosedLoopError(0));
		//signal = ( leftFront.getMotorOutputPercent() + rightFront.getMotorOutputPercent() ) / 2;
		if(error < .05) {
			return true;
		}
		leftFront.set(ControlMode.Position, target);
		rightFront.set(ControlMode.Position, target);
		return false;
	}
	
	/**
	 * Method to drive a certain distance using time.
	 * @param target Desired distance.
	 * @return True if the robot has arrived at the target, false otherwise.
	 */
	public boolean driveToTimed(double target) {
		target = target * TIME_CONVERSION; // time it takes to travel one inch at .4 power, measured in milliseconds
		
		timer.delay(target);
		drive.tankDrive(.4, .4);
		
		return true;
	}

	
	/**
	 * Method to turn to a certain angle using MXP.
	 * @param target Desired angle.
	 * @return True if the robot has arrived at the target, false otherwise.
	 */
	public boolean turnTo(double target) {
		if(elevatorZeroed) {
			if( (Math.abs(imu.getAngle()) > Math.abs(target) + 10 || Math.abs(imu.getAngle()) < Math.abs(target) - 10) ) {
				if(Math.abs(target - imu.getAngle()) * .035 < .5){
					drive.arcadeDrive(0, (target - imu.getAngle()) * .035);	
				}
				else if(target - imu.getAngle() < 0){
					drive.arcadeDrive(0, -.5);	
				}
				else {
					drive.arcadeDrive(0, .5);
				}
				return false;
			}
			else {
				drive.tankDrive(0.0, 0.0);
				timer.delay(.5);
				resetEncoders();
				resetGyro();
				return true;
			}
		}
		else {
			if( (Math.abs(imu.getAngle()) > Math.abs(target) + 10 || Math.abs(imu.getAngle()) < Math.abs(target) - 10) ) {
				if(Math.abs(target - imu.getAngle()) * .02 < .45){
					drive.arcadeDrive(0, (target - imu.getAngle()) * .02);	
				}
				else if(target - imu.getAngle() < 0){
					drive.arcadeDrive(0, -.45);	
				}
				else {
					drive.arcadeDrive(0, .45);
				}
				return false;
			}
			else {
				drive.tankDrive(0.0, 0.0);
				timer.delay(.5);
				resetEncoders();
				resetGyro();
				return true;
			}
		}
	}
	
	/**
	 * Method to move the elevator a certain distance using PID.
	 * @param target Desired distance.
	 * @return True if the elevator has arrived at the target, false otherwise.
	 */
	public boolean elevatorTo(double target) {
		if( error< .05) {
			return true;
		}
		elevator.set(ControlMode.Position, target); 
		error = (elevator.getClosedLoopError(0)); 
		return false; 
	}
	
	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();	
	}
	
	public void resetGyro(){
		imu.resetDisplacement();
		imu.reset();
	}
	
	public void dummyAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(168 - ROBOT_HALF)) {
			}
			else {
				stage = AutoStage.end;
			}
			break;
		case end:
			break;
		default:
			break;
		}
	}

	public void leftLeftSwAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(168 - ROBOT_HALF)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(30 - ROBOT_HALF)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			//if(gameData.substring(0,1).equals("L")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			//}
			stage = AutoStage.end;
			break;
		case end:
			break;
		default:
			break;
		}
	}


	public void leftRightSwAuto() {
		switch(stage) {
		case turnZero:
				stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(225)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(250)) {
			}
			else {
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(!driveTo(30)) {
			}
			else {
				stage = AutoStage.turnThree;
			}
			break;
		case turnThree:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveFour;
			}
			break;
		case driveFour:
			if(!driveTo(5)) {
			}
			else {
				stage = AutoStage.place;
			}
				
		case place:
			if(gameData.substring(0,1).equals("R")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			}
				stage = AutoStage.end;
			break;
		case end:
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
			if(!driveTo(332 - ROBOT_HALF)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(7)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			//if(gameData.substring(1,2).equals("L")) {
				timer.delay(.5);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			//}
			stage = AutoStage.end;
			break;
		case end:
			break;
		default:
			break;
		}
	}

	public void leftRightScAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(225)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(260)) {
			}
			else {
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(!driveTo(60)) {
			}
			else {
				stage = AutoStage.turnThree;
			}
			break;
		case turnThree:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveFour;
			}
			break;
		case driveFour:
			if(!driveTo(5)) {
			}
			else {
				stage = AutoStage.place;
			}
		case place:
			if(gameData.substring(1,2).equals("R")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			}
			stage = AutoStage.end;
			break;
		case end:
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
			if(!driveTo(168 - ROBOT_HALF)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(30 - ROBOT_HALF)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			//if(gameData.substring(0,1).equals("R")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			//}
			stage = AutoStage.end;
			break;
		case end:
			break;
		default:
			break;
		}
	}

	public void rightLeftSwAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(225)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(250)) {
			}
			else {
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(!driveTo(30)) {
			}
			else {
				stage = AutoStage.turnThree;
			}
			break;
		case turnThree:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveFour;
			}
			break;
		case driveFour:
			if(!driveTo(5)) {
			}
			else {
				stage = AutoStage.place;
			}

		case place:
			if(gameData.substring(0,1).equals("L")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			}
			stage = AutoStage.end;
			break;
		case end:
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
			if(!driveTo(332 - ROBOT_HALF)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(7)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			//if(gameData.substring(1,2).equals("R")) {
				timer.delay(.5);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			//}
			stage = AutoStage.end;
			break;
		case end:
			break;
		default:
			break;
		}
	}

	public void rightLeftScAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(225)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(260)) {
			}
			else {
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(!driveTo(60)) {
			}
			else {
				stage = AutoStage.turnThree;
			}
			break;
		case turnThree:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveFour;
			}
			break;
		case driveFour:
			if(!driveTo(5)) {
			}
			else {
				stage = AutoStage.place;
			}
		case place:
			if(gameData.substring(1,2).equals("L")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			}
			stage = AutoStage.end;
			break;
		case end:
			break;
		default:
			break;
		}
	}

	public void centerLeftSwAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(60)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(-43.315)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(96.208)) {
			}
			else {
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(!turnTo(43.315)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			if(gameData.substring(0,1).equals("L")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			}
			stage = AutoStage.end;
			break;
		case end:
			break;
		default:
			break;
		}
	}
	
	public void centerRightSwAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(60)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(43.315)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(96.208)) {
			}
			else {
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(!turnTo(-43.315)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			if(gameData.substring(0,1).equals("R")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			}
			stage = AutoStage.end;
			break;
		case end:
			break;
		default:
			break;
		}
	}
	
	public void centerLeftScAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(60)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(-52)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(114.809)) {
			}
			else {
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(!turnTo(52)) {
			}
			else {
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(!driveTo(156)) {
			}
			else {
				stage = AutoStage.turnThree;
			}
			break;
		case turnThree:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			if(gameData.substring(1,2).equals("L")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			}
			stage = AutoStage.end;
			break;
		case end:
			break;
		default:
			break;
		}
	}
	
	public void centerRightScAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(60)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(52)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(114.809)) {
			}
			else {
				stage = AutoStage.turnTwo;
			}
			break;
		case turnTwo:
			if(!turnTo(-52)) {
			}
			else {
				stage = AutoStage.driveThree;
			}
			break;
		case driveThree:
			if(!driveTo(156)) {
			}
			else {
				stage = AutoStage.turnThree;
			}
			break;
		case turnThree:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			if(gameData.substring(1,2).equals("R")) {
				liftSully.set(DoubleSolenoid.Value.kForward);
				timer.delay(1);
				leftIntake.set(1.0);
				rightIntake.set(-1.0);
				timer.delay(1);
				leftIntake.set(0.0);
				rightIntake.set(0.0);
			}
			stage = AutoStage.end;
			break;
		case end:
			break;
		default:
			break;
		}
	}
}