// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;

public class Autonomus2 extends Command {

  
	private PIDController linearControllerl = new PIDController(0.003, 0.00025, 0.0001);
	private PIDController linearControllerr = new PIDController(0.003, 0.00025, 0.0001);

  private Chassis m_Chassis;

  public Autonomus2() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_Chassis);

    m_Chassis = Robot.m_Chassis;

		linearControllerl.setIntegratorRange(-5, 5);
		linearControllerr.setIntegratorRange(-5,5);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    linearActuatePIDReset();
  }


  double rotToNative() {
    return 360*4.0/3.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    moveLinearPID(rotToNative()*4);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}

//i implimentation functions
  


	private double calibratedStartLeft = 0;
	private double calibratedStartRight = 0;
	
	public void linearActuatePIDReset() {
		calibratedStartLeft = m_Chassis.getLeftEncoderPosition();
		calibratedStartRight = m_Chassis.getRightEncoderPosition();
		linearControllerl.reset();
		linearControllerr.reset();
	}



  public void moveLinearPID(double distance) {
		double maxSpeed = 0.15;
		double lspeed = -MathUtil.clamp(linearControllerl.calculate(-(m_Chassis.getLeftEncoderPosition() - calibratedStartLeft), distance),-maxSpeed,maxSpeed);
		double rspeed = MathUtil.clamp(linearControllerr.calculate(m_Chassis.getRightEncoderPosition() - calibratedStartRight, distance),-maxSpeed,maxSpeed);

		SmartDashboard.putNumber("leftDriveError", linearControllerl.getPositionError());
		SmartDashboard.putNumber("RightDriveError", linearControllerr.getPositionError());
		// ADIS16448_IMU
		m_Chassis.setLeft(lspeed);
		m_Chassis.setRight(rspeed);
	}

}
