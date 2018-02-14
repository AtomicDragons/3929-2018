/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3929.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
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
	
	//Motor controllers
	WPI_TalonSRX leftFront;
	WPI_VictorSPX leftRear;
	WPI_TalonSRX rightFront;
	WPI_VictorSPX rightRear;
	WPI_TalonSRX elevator;
	
	Spark leftIntake;
	Spark rightIntake;	
	WPI_TalonSRX frontHanger;
	VictorSP backHanger; 
	
	
	//Sensors
	DigitalInput elevatorLimitSwitch;
	
	//Controller values
	private double error;
	private double signal;
	
	VictorSP light;
	
	//Controller groups
	SpeedControllerGroup leftMotors;
	SpeedControllerGroup rightMotors;
	
	//Drive
	DifferentialDrive drive;
	double leftPow;
	double rightPow;
	double drivePow;

	//Constants
	final double robotHalf = 20; //Half the length of the robot
	final double robotLength = 40; //The length of the robot
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
	private boolean elevatorZeroed;
	AutoStage stage;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		imu = new AHRS(SPI.Port.kMXP);
		
		RobotSchematic map = new RobotSchematic();
		
		//leftEncoder.setDistancePerPulse(10); // wheel diameter * Math.PI);
		
		//Initialize motor controllers
		leftFront = new WPI_TalonSRX(map.LEFT_FRONT_MOTOR_PORT);
		leftRear = new WPI_VictorSPX(map.LEFT_REAR_MOTOR_PORT);
		leftRear.follow(leftFront);
		rightFront = new WPI_TalonSRX(map.RIGHT_FRONT_MOTOR_PORT);
		rightRear = new WPI_VictorSPX(map.RIGHT_REAR_MOTOR_PORT);
		rightRear.follow(rightFront);
		
		elevator = new WPI_TalonSRX(map.ELEVATOR_MOTOR_PORT);
		frontHanger = new WPI_TalonSRX(map.FRONT_HANGER_MOTOR_PORT); 
		backHanger = new VictorSP(map.REAR_HANGER_MOTOR_PORT);
		
		leftIntake = new Spark(map.LEFT_INTAKE_MOTOR_PORT);
		rightIntake = new Spark(map.RIGHT_INTAKE_MOTOR_PORT);
		
		
		//Initialize/configures sensors
		// elevatorLimitSwitch = new DigitalInput();
		leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
		elevatorZeroed = false;
		
		//Add motor controllers to control groups
		leftMotors = new SpeedControllerGroup(leftFront);
		rightMotors = new SpeedControllerGroup(rightFront);
		
		//Add controller groups to differential drive
		drive = new DifferentialDrive(leftMotors, rightMotors);
		
		//Initialize controllers
		driveStick = new Joystick(0);
		opStick = new Joystick(1);
		
		//Add data to SmartDashboard
		signal = 0.0;
		error = 0.0;
		
		SmartDashboard.putNumber("Signal", signal);
		SmartDashboard.putNumber("Error", error);
		
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
		
		dAngle = imu.getFusedHeading() - angle;
		angle = imu.getFusedHeading();
		
		if(!elevatorZeroed) {
			if(elevatorZero() ) {	
				elevatorZeroed = false;
			}
		}
		
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

		/*if(driveStick.getRawButton(1)){
			leftEncoder.reset();
			rightEncoder.reset();
		} 
		
		if(driveStick.getRawButton(2)){
			imu.resetDisplacement();
		}*/ //commented out for eagles demo
		
		//GTA
		signal = ( leftFront.getMotorOutputPercent() + rightFront.getMotorOutputPercent() ) / 2;
		switch (m_driveSelected) {
			
			//Arcade
			case kArcadeMode:
			drive.arcadeDrive(driveStick.getRawAxis(1), driveStick.getRawAxis(0), true);
			break;
			
			//Tank
			case kTankMode:
			drive.tankDrive(driveStick.getRawAxis(1) * -1, driveStick.getRawAxis(5) * -1, true);
			break;
			
			//GTA
			case kConsoleMode:
			drive.arcadeDrive(
			Math.pow((driveStick.getRawAxis(2) * (-1)) + driveStick.getRawAxis(3), 3),
			Math.pow((driveStick.getRawAxis(0) * (1)), 3), true);
			break;
			
			default:
			break;
		}
		
		if(opStick.getRawButton(0)) {
			leftIntake.set(-1.0);
			rightIntake.set(1.0);
		}
		
		elevator.set(opStick.getRawAxis(1) * -1);
		
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
	public boolean elevatorZero() {
		if(elevatorLimitSwitch.get()) {
			elevator.set(-.2);
			return false;
		}
		else {
			return true; 
		}
	}
	
	/**
	 * Method to drive a certain distance using PID.
	 * @param target Desired distance.
	 * @return True if the robot has arrived at the target, false otherwise.
	 */
	public boolean driveTo(double target) {
		//remember to set coefficients kP, kI, & kD
		error = ( leftFront.getClosedLoopError(0) + rightFront.getClosedLoopError(0) ) / 2;
		signal = ( leftFront.getMotorOutputPercent() + rightFront.getMotorOutputPercent() ) / 2;
		if(error < .05) {
			return true;
		}
		leftFront.set(ControlMode.Position, target);
		rightFront.set(ControlMode.Position, target);
		return false;
	}
	
	/**
	 * Method to turn to a certain angle using MXP.
	 * @param target Desired angle.
	 * @return True if the robot has arrived at the target, false otherwise.
	 */
	public boolean turnTo(double target) {
		double proportion = ( Math.abs(target) - Math.abs(angle) ) / Math.abs(target);
		double derivative = dAngle;
		double power = 1;
		int dir = 1;
		if(target < angle) {
			dir = -1;
		}
		if(angle >= target) {
			resetGyro();
			return true;
		}
		drive.tankDrive(dir * power, dir * power * -1);
		return false;
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
	
	public void resetGyro(){
		imu.resetDisplacement();
	}
	
	public void leftLeftSwAuto() {
		switch(stage) {
		case turnZero:
			stage = AutoStage.driveOne;
			break;
		case driveOne:
			if(!driveTo(168 - robotHalf)) {
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
			if(!driveTo(85.25 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			break;
		default:
			break;
		}
	}


	public void leftRightSwAuto() {
		switch(stage) {
		case turnZero:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(!driveTo(264 - robotHalf)) {
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
			if(!driveTo(168 - robotHalf )) {
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
			if(!driveTo(85.25 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
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
			if(!driveTo(324 - robotHalf)) {
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
			if(!driveTo(71.57 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			break;
		default:
			break;
		}
	}

	public void leftRightScAuto() {
		switch(stage) {
		case turnZero:
			if(!turnTo(90)) {
			}
			else {
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(!driveTo(264 - robotHalf)) {
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
			if(!driveTo(324 - robotHalf)) {
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
			if(!driveTo(71.57 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
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
			if(!driveTo(168 - robotHalf)) {
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
			if(!driveTo(85.25 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			break;
		default:
			break;
		}
	}

	public void rightLeftSwAuto() {
		switch(stage) {
		case turnZero:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(!driveTo(264 - robotHalf)) {
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
			if(!driveTo(168 - robotHalf )) {
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
			if(!driveTo(85.25 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
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
			if(!driveTo(324 - robotHalf)) {
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
			if(!driveTo(71.57 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			break;
		default:
			break;
		}
	}

	public void rightLeftScAuto() {
		switch(stage) {
		case turnZero:
			if(!turnTo(-90)) {
			}
			else {
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(!driveTo(264 - robotHalf)) {
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
			if(!driveTo(324 - robotHalf)) {
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
			if(!driveTo(71.57 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			break;
		default:
			break;
		}
	}

	public void centerLeftSwAuto() {
		switch(stage) {
		case turnZero:
			if(!turnTo(-43.95)) {
			}
			else {
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(!driveTo(233.38 - robotHalf)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(133.95)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(85.25 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			break;
		default:
			break;
		}
	}
	
	public void centerRightSwAuto() {
		switch(stage) {
		case turnZero:
			if(!turnTo(43.95)) {
			}
			else {
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(!driveTo(233.38 - robotHalf)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(-133.95)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(85.25 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
			break;
		case place:
			break;
		default:
			break;
		}
	}
	
	public void centerLeftScAuto() {
		switch(stage) {
		case turnZero:
			if(!turnTo(-43.95)) {
			}
			else {
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(!driveTo(233.38 - robotHalf)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(43.95)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(156 - robotHalf)) {
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
			if(!driveTo(71.57 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
		case place:
			break;
		default:
			break;
		}
	}
	
	public void centerRightScAuto() {
		switch(stage) {
		case turnZero:
			if(!turnTo(43.95)) {
			}
			else {
				stage = AutoStage.driveOne;
			}
			break;
		case driveOne:
			if(!driveTo(233.38 - robotHalf)) {
			}
			else {
				stage = AutoStage.turnOne;
			}
			break;
		case turnOne:
			if(!turnTo(-43.95)) {
			}
			else {
				stage = AutoStage.driveTwo;
			}
			break;
		case driveTwo:
			if(!driveTo(156 - robotHalf)) {
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
			if(!driveTo(71.57 - robotLength)) {
			}
			else {
				stage = AutoStage.place;
			}
		case place:
			break;
		default:
			break;
		}
	}
}
