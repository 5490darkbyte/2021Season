/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// The following mapping is final as of 2/22/18
	// the following are CAN addresses for the Talon SRX's
	/*
	public static int mtrFrontLeft = 1;
	public static int mtrFrontRight = 2;
	public static int mtrRearRight = 3;
	public static int mtrRearLeft = 4;	
	*/
	public static int backLeftDrive = 1;
	public static int midLeftDrive = 2;
	public static int frontLeftDrive = 3;

	public static int backRightDrive = 4;
	public static int midRightDrive = 5;
	public static int frontRightDrive = 6;

	// small motor on collector. Not sure what ports to use
	// public static int collectorWheels = 7;

	// the following are CAN addresses for the SparkMAX
	/* Left side of the robot, back to front */
	public static int shooter1 = 11;
	public static int collector = 12; // has winch port for now
	public static int shooter2  = 13;

	/* Right side of the robot, back to front */
	public static int collectorWheels = 14;
	public static int lowerChute = 15;
	public static int upperChute = 16;	
	
	// the following are CAN addresses for the Spark's	 
	

	// PWM
	public static int mtrHatch = 0;
	
	// IMU Reservations
	// The IMU is located on the RoboRio MXP port and uses some of the DIO channels
	// this means that the SPI
	//
	// m_interrupt - MXP DIO0 -> DIO10 - IMU interrupt
	// m_reset = MXP DIO8 -> DIO18 - IMU reset
	// MOSI = MXP DIO7 -> DIO17 - SPI MOSI
	// MISO = MXP DIO6 -> DIO16 - SPI MISO
	// CLK = MXP DIO5 -> DIO15 - SPI CLK
	// CS = MXP DIO4 -> DIO14 - SPI CS
	
	// reserve these too..
	// I2C SDA = MXP DIO 15 - DIO25
	// I2C SCL = MXP DIO 14 - DIO24
	
	// MXP available (encoder)
	// pin used
	// MXP DIO1   -> DIO11/PWM11  (gripper ls)
	// MXP DIO2   -> DIO12/PWM12  (gripper ls)
	// MXP DIO3   -> DIO13/PWM13
	// block used...
	// MXP DIO9   -> DIO19/PWM15
	// MXP DIO10  -> DIO20/PWM16
	// MXP DIO11  -> DIO21/PWM17	
	// MXP DIO12  -> DIO22/PWM18  
	// MXP DIO13  -> DIO23/PWM19
	
	// Analog outputs AO0, AO1
	// Analog inputs AI4, AI5, AI6, AI7
	
	
	
	
	// Encoder map
	public static int leftEncoder = 22;
	public static int rightEncoder = 21;	
	public static int collectorEncoder = 23;
		
	

}
