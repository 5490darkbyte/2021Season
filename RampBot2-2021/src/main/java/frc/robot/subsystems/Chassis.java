package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.commands.LiftManualMove;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// this is for the Talon SRX version
//import edu.wpi.first.wpilibj.Talon;
//import com.revrobotics.*;


import frc.robot.RobotMap;
import frc.robot.commands.DriveRobot;

//import frc.robot.Point3D;





import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.AnalogInput;


//import com.analog.adis16448.frc.ADIS16448_IMU;
import frc.robot.subsystems.TransferTable;
//import jdk.internal.jshell.tool.resources.l10n;





/**
 *
 */
public class Chassis extends Subsystem {
	
	// lower limit for speed setting  
	private static final double minimum_drive = 0.1;
	
	// Main Movement Drive 
	
	WPI_TalonSRX motorFrontLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX motorMidLeft = new WPI_TalonSRX(2);
	WPI_TalonSRX motorRearLeft = new WPI_TalonSRX(3);
	SpeedControllerGroup m_left = new SpeedControllerGroup(motorFrontLeft, motorMidLeft, motorRearLeft);

	WPI_TalonSRX motorFrontRight = new WPI_TalonSRX(4);
	WPI_TalonSRX motorMidRight = new WPI_TalonSRX(5);
	WPI_TalonSRX motorRearRight = new WPI_TalonSRX(6);
	SpeedControllerGroup m_right = new SpeedControllerGroup(motorFrontRight, motorMidRight, motorRearRight);

	DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
	
	// Set the drive to whichever one we are using.  For 2021 we use Tank
    
	// Set the drive to whichever one we are using.  For 2021 we use Tank
    
	
	AnalogInput  LeftDistance = new AnalogInput(RobotMap.us_LeftDist);
	AnalogInput  RightDistance = new AnalogInput(RobotMap.us_RightDist);

    
    
    public int segment;
    public double percent;
    public double tick; 
    
    public double speed;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	// see DriveTrain example
    public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		
		    /* factory default values */
		/*motorFrontLeft.configFactoryDefault();
		motorRearLeft.configFactoryDefault();
		motorFrontRight.configFactoryDefault();
		motorRearRight.configFactoryDefault();*/

		
		//int bits;
		LeftDistance.setOversampleBits(4);
		//bits = LeftDistance.getOversampleBits();
		LeftDistance.setAverageBits(2);
		//bits = LeftDistance.getAverageBits();

		RightDistance.setOversampleBits(4);
		//bits = LeftDistance.getOversampleBits();
		RightDistance.setAverageBits(2);
		//bits = LeftDistance.getAverageBits();


		
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
		

		SmartDashboard.putNumber("Left Distance: ",  LeftDistance.getValue());
		SmartDashboard.putNumber("Right Distance: ",  RightDistance.getValue());
		
		

    }
    
   
    public void log() 
    {
    	
    	// put class variables we want to see on dashboard or capture here
    	
    	
    	SmartDashboard.putNumber("FL", -1 * motorFrontLeft.get());
    	SmartDashboard.putNumber("FR", motorFrontRight.get());
    	SmartDashboard.putNumber("RL", -1 * motorRearLeft.get());
    	SmartDashboard.putNumber("RR", motorRearRight.get());

		SmartDashboard.putNumber("Path Segment: ", segment);
		SmartDashboard.putNumber("Path percent: ", percent);
		SmartDashboard.putNumber("Path timer: ", tick);

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

		m_robotDrive.arcadeDrive(speed*driveStick.getY(), speed*driveStick.getX());
									
		
		//
		
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
	
	public void moveBackward()
	{
		m_robotDrive.tankDrive(-1,0);
	}

	public void hatchOpen()
	{
		
	}

	public void hatchClose()
	{
			//0.45
		//0.8
	
	}
}


