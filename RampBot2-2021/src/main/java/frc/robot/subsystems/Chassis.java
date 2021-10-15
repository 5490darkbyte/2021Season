package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.MySpeedControllerGroup;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;

// import frc.robot.commands.LiftManualMove;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

// this is for the Talon SRX version
//import edu.wpi.first.wpilibj.Talon;
//import com.revrobotics.*;


import frc.robot.RobotMap;
import frc.robot.commands.DriveRobot;

//import frc.robot.Point3D;





import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;


//import com.analog.adis16448.frc.ADIS16448_IMU;

//import jdk.internal.jshell.tool.resources.l10n;





/**
 *
 */
public class Chassis extends Subsystem {
	
	// lower limit for speed setting  
	private static final double minimum_drive = 0.1;
	
	// Main Movement Drive 
	
	WPI_TalonSRX motorFrontLeft = new WPI_TalonSRX(RobotMap.frontLeftDrive);
	WPI_TalonSRX motorMidLeft = new WPI_TalonSRX(RobotMap.midLeftDrive);
	WPI_TalonSRX motorRearLeft = new WPI_TalonSRX(RobotMap.backLeftDrive);
	WPI_TalonSRX[] leftControllers = {motorFrontLeft, motorMidLeft, motorRearLeft};
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftControllers);
	

	WPI_TalonSRX motorFrontRight = new WPI_TalonSRX(RobotMap.frontRightDrive);
	WPI_TalonSRX motorMidRight = new WPI_TalonSRX(RobotMap.midRightDrive);
	WPI_TalonSRX motorRearRight = new WPI_TalonSRX(RobotMap.backRightDrive);
	WPI_TalonSRX[] rightControllers = {motorFrontRight, motorMidRight, motorRearRight};
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightControllers);

	//CANCoder driveEncoders = new CANCoder(RobotMap.leftEncoder, RobotMap.rightEncoder);
	CANCoder leftDriveEncoder = new CANCoder(RobotMap.leftEncoder);
	CANCoder rightDriveEncoder = new CANCoder(RobotMap.rightEncoder);

	DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
	
	PIDController linearControllerl = new PIDController(0.003, 0.00025, 0.0001);
	PIDController linearControllerr = new PIDController(0.003, 0.00025, 0.0001);

	public Chassis() {
		// m_left.setInverted(true);

		linearControllerl.setIntegratorRange(-5, 5);
		linearControllerr.setIntegratorRange(-5,5);
	}

	// Set the drive to whichever one we are using.  For 2021 we use Tank
    
	// Set the drive to whichever one we are using.  For 2021 we use Tank
	
	// Dr. Lawlis said "Analog Input" is for position sensors, may be relevant for autonomous but not now
	/*
	AnalogInput  LeftDistance = new AnalogInput(1);
	AnalogInput  RightDistance = new AnalogInput(2);
	*/
	
	
    public int segment;
    public double percent;
    public double tick; 
    
    public double speed;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	// see DriveTrain example

	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		super.periodic();

		plotPoses();
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		
			/* factory default values */
		leftDriveEncoder.setPosition(0.0);
		rightDriveEncoder.setPosition(0.0);

		leftDriveEncoder.configVelocityMeasurementWindow(32, 0);
		rightDriveEncoder.configVelocityMeasurementWindow(32, 0);

		
		/*motorFrontLeft.configFactoryDefault();
		motorRearLeft.configFactoryDefault();
		motorFrontRight.configFactoryDefault();
		motorRearRight.configFactoryDefault();*/
		
		//Use this code to calibrate the motors
		
		/*
		int bits;
		LeftDistance.setOversampleBits(4);
		bits = LeftDistance.getOversampleBits();
		LeftDistance.setAverageBits(2);
		bits = LeftDistance.getAverageBits();

		RightDistance.setOversampleBits(4);
		bits = LeftDistance.getOversampleBits();
		RightDistance.setAverageBits(2);
		bits = LeftDistance.getAverageBits();
		*/
		

		
		//When no other command is running let the operator drive around using the joystick		 
		setDefaultCommand(new DriveRobot());
		
		
		/*
		SmartDashboard.putNumber("Gyro-X", imu.getAngleX());
		SmartDashboard.putNumber("Gyro-Y", imu.getAngleY());
		SmartDashboard.putNumber("Gyro-Z", imu.getAngleZ());
		
		SmartDashboard.putNumber("Accel-X", imu.getAccelX());
		SmartDashboard.putNumber("Accel-Y", imu.getAccelY());
		SmartDashboard.putNumber("Accel-Z", imu.getAccelZ());
		
		SmartDashboard.putNumber("Pitch", imu.getPitch());
		SmartDashboard.putNumber("Roll", imu.getRoll());
		SmartDashboard.putNumber("Yaw", imu.getYaw());
		
		SmartDashboard.putNumber("Pressure: ", imu.getBarometricPressure());
		SmartDashboard.putNumber("Temperature: ", imu.getTemperature());
		*/
		

		/* For AnalogInput, which we aren't using?
		// SmartDashboard.putNumber("Left Distance: ", LeftDistancePot.get());
		SmartDashboard.putNumber("Left Distance: ",  LeftDistance.getValue());
		SmartDashboard.putNumber("Right Distance: ",  RightDistance.getValue());
		*/
		
		

    }
    
   
    public void log() 
    {
    	
    	// put class variables we want to see on dashboard or capture here
    	
    	/*
    	SmartDashboard.putNumber("FL", -1 * motorFrontLeft.get());
    	SmartDashboard.putNumber("FR", motorFrontRight.get());
    	SmartDashboard.putNumber("RL", -1 * motorRearLeft.get());
    	SmartDashboard.putNumber("RR", motorRearRight.get());

		SmartDashboard.putNumber("Path Segment: ", segment);
		SmartDashboard.putNumber("Path percent: ", percent);
		SmartDashboard.putNumber("Path timer: ", tick);*/

		/*
		SmartDashboard.putNumber("Gyro-X", imu.getAngleX());
		SmartDashboard.putNumber("Gyro-Y", imu.getAngleY());
		SmartDashboard.putNumber("Gyro-Z", imu.getAngleZ());
		
		SmartDashboard.putNumber("Accel-X", imu.getAccelX());
		SmartDashboard.putNumber("Accel-Y", imu.getAccelY());
		SmartDashboard.putNumber("Accel-Z", imu.getAccelZ());
		
		SmartDashboard.putNumber("Pitch", imu.getPitch());
		SmartDashboard.putNumber("Roll", imu.getRoll());
		SmartDashboard.putNumber("Yaw", imu.getYaw());
		
		SmartDashboard.putNumber("Angle-X", imu.getAngleX());
		SmartDashboard.putNumber("Angle-Y", imu.getAngleY());
		SmartDashboard.putNumber("Angle-Z", imu.getAngleZ());
		
		SmartDashboard.putNumber("Pressure: ", imu.getBarometricPressure());
		SmartDashboard.putNumber("Temperature: ", imu.getTemperature());
		*/
		
		SmartDashboard.putNumber("Speed: ", speed);
		
	}

	
	public void Drive(Joystick driveStick)
	{
		double speedrange = 1 - minimum_drive;
		speed = (-speedrange*driveStick.getThrottle()+1)/2;
		speed += minimum_drive;

		//m_robotDrive.setDeadband(0.2);
		
		/*
		
		m_robotDrive.driveCartesian(Xout, Yout, Zout, 0);
		*/
		
		m_robotDrive.arcadeDrive(speed*driveStick.getY(), -speed*driveStick.getX());
		
		
	}
	
	// Let an external function drive the chassis
	// note X = forward, Y right 
	// 
	public void Drive(double X, double Y, double speed)
	{
		// mechanum orientation is
		//  X forward/reverse
		//	Y right/left
		//  Z twist		
		// normal people assume
		//  Y forward/reverse
		//	X right/left
		//  Z twist
		//  So we swap orientations here to avoid brain cramps..
		//
		if (X > 1.0) X = 1.0;
		if (Y > 1.0) Y = 1.0;
		if (speed > 1.0) speed = 1.0;
		
		m_robotDrive.arcadeDrive(Y*speed, X*speed);
		
	}

	
	public void StopMotors()
	{		
		m_robotDrive.arcadeDrive(0,0);		
	}
	
	public void moveForward()
	{
		m_robotDrive.arcadeDrive(1, 0);
	}
	public void moveForward(double speed)
	{
		m_robotDrive.arcadeDrive(1*speed, 0);
		SmartDashboard.putNumber("Velocity Left", leftDriveEncoder.getVelocity());
		SmartDashboard.putNumber("Velocity Right", rightDriveEncoder.getVelocity());
		SmartDashboard.putNumber("Absolute Position", leftDriveEncoder.getAbsolutePosition());
		SmartDashboard.putNumber("Velocity", leftDriveEncoder.getVelocity());

	}
	
	public void move1Side(double speed, boolean leftSide)
	{
		if (leftSide)
		{
			m_left.set(speed);
		}
		else
		{
			m_right.set(speed);
		}
		
	}

	public void moveBackward()
	{
		m_robotDrive.arcadeDrive(-1, 0);
	}
	public void moveBackward(double speed)
	{
		m_robotDrive.tankDrive(-1*speed,0);
	}

	public void moveLeftForward(double speed)
	{
		SmartDashboard.putNumber("Left side velocity", leftDriveEncoder.getVelocity());
		m_left.set(speed);
	}
	public void moveRightForward(double speed)
	{
		m_right.set(speed);
	}

	// public void hatchOpen()
	// {
		
	// }

	public double getLeftVelocity()
	{
		return leftDriveEncoder.getVelocity();
	}
	public double getRightVelocity()
	{
		return rightDriveEncoder.getVelocity();
	}

	public void hatchClose()
	{
			//0.45
		//0.8
	
	}

	private double calibratedStartLeft = 0;
	private double calibratedStartRight = 0;
	
	public void linearActuatePIDReset() {
		calibratedStartLeft = leftDriveEncoder.getPosition();
		calibratedStartRight = rightDriveEncoder.getPosition();
		linearControllerl.reset();
		linearControllerr.reset();
	}

	public void moveLinearPID(double distance) {
		double maxSpeed = 0.15;
		double lspeed = -MathUtil.clamp(linearControllerl.calculate(-(leftDriveEncoder.getPosition() - calibratedStartLeft), distance),-maxSpeed,maxSpeed);
		double rspeed = MathUtil.clamp(linearControllerr.calculate(rightDriveEncoder.getPosition() - calibratedStartRight, distance),-maxSpeed,maxSpeed);

		SmartDashboard.putNumber("leftDriveError", linearControllerl.getPositionError());
		SmartDashboard.putNumber("RightDriveError", linearControllerr.getPositionError());
		// ADIS16448_IMU
		m_left.set(lspeed);
		m_right.set(rspeed);
	}

	public void plotPoses() {
		// for forward left spin CCW and right spin CW	
		SmartDashboard.putNumber("leftDrivePos", leftDriveEncoder.getPosition());
		SmartDashboard.putNumber("RightDrivePos", rightDriveEncoder.getPosition());
		
		SmartDashboard.putNumber("leftDriveVel", leftDriveEncoder.getVelocity());
		SmartDashboard.putNumber("RightDriveVel", rightDriveEncoder.getVelocity());
	}

	public double getBusVoltage()
	{
		return rightDriveEncoder.getBusVoltage();
	}

}