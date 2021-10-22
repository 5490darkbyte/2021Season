// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;

public class Autonomus2 extends Command {

  
	private PIDController linearControllerl = new PIDController(0.003, 0.00025, 0.0001);
  private PIDController linearControllerr = new PIDController(0.003, 0.00025, 0.0001);
  
  private PIDController scurveControllerl = new PIDController(0.003, 0.00025, 0*0.0001);
	private PIDController scurveControllerr = new PIDController(0.003, 0.00025, 0*0.0001);

  private Chassis m_Chassis;

  public Autonomus2() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_Chassis);

    m_Chassis = Robot.m_Chassis;

		linearControllerl.setIntegratorRange(-5, 5);
    linearControllerr.setIntegratorRange(-5,5);
    
    

		scurveControllerl.setIntegratorRange(-5, 5);
		scurveControllerr.setIntegratorRange(-5,5);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    linearActuatePIDReset();
    startCurvePID(20, 8, rotToNative()*4);;
  }


  double rotToNative() {
    return 360*4.0/3.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    moveLinearPID(rotToNative()*4);
    // runSCurveIncriment();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_Chassis.setLeft(0);
    m_Chassis.setRight(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}

//i implimentation functions
  


	private double lcalibratedStartLeft = 0;
  private double lcalibratedStartRight = 0;
  
	private double scalibratedStartLeft = 0;
	private double scalibratedStartRight = 0;
	
	public void linearActuatePIDReset() {
		lcalibratedStartLeft = m_Chassis.getLeftEncoderPosition();
		lcalibratedStartRight = m_Chassis.getRightEncoderPosition();
		linearControllerl.reset();
    linearControllerr.reset();
    
    
	}

  public void scurvePIDReset() {
		scalibratedStartLeft = m_Chassis.getLeftEncoderPosition();
		scalibratedStartRight = m_Chassis.getRightEncoderPosition();
		scurveControllerl.reset();
		scurveControllerr.reset();
	}

  public void checkIfLinearComplete() {
    
  }

  public void moveLinearPID(double distance) {
		double maxSpeed = 0.15;
		double lspeed = -MathUtil.clamp(linearControllerl.calculate(-(m_Chassis.getLeftEncoderPosition() - lcalibratedStartLeft), distance),-maxSpeed,maxSpeed);
		double rspeed = MathUtil.clamp(linearControllerr.calculate(m_Chassis.getRightEncoderPosition() - lcalibratedStartRight, distance),-maxSpeed,maxSpeed);

		SmartDashboard.putNumber("leftDriveError", linearControllerl.getPositionError());
		SmartDashboard.putNumber("RightDriveError", linearControllerr.getPositionError());
		// ADIS16448_IMU
		m_Chassis.setLeft(lspeed);
		m_Chassis.setRight(rspeed);
  }

  private int StepINcrimentRate = 1;

  private double currentStepSetTime;
  private int currentStep = 0;
  private int totalSteps = 0;
  private double startTime;
  private double priviusFrame;
  private double totalDuration;
  private double totalDistance;


  public void startCurvePID(int totalSteps,double totalTime,double totalDistance) {
    scurvePIDReset();
    this.totalSteps = totalSteps;
    startTime = Timer.getFPGATimestamp();
    priviusFrame = startTime;
    this.totalDuration = totalTime;
    this.totalDistance = totalDistance;
    currentStep = 0;
    currentStepSetTime = startTime;
  }

  private double positionForTime(double normalizedT,double totalDistance) {


    double scaledTime = 6*Math.pow(normalizedT, 5)-15*Math.pow(normalizedT, 4)+10*Math.pow(normalizedT, 3);

    return scaledTime * totalDistance;
  }

  /// returns weather completeed - right now when true not actually completed
  public boolean runSCurveIncriment() {



    double now = Timer.getFPGATimestamp();

    double stepTime = totalDuration / totalSteps;

    if (now - currentStepSetTime >= stepTime) {
     

      if (currentStep == totalSteps - 1) {
        // return true;
      }
      else {
        currentStep += 1;
        currentStepSetTime = now;
      }
    }

    
    double setPos = positionForTime(((double)(currentStep + 1) / (double)(totalSteps)), totalDistance);
    
    moveScurvePID(setPos);

    SmartDashboard.putNumber("SCurveUpdateDeltaTime", currentStep);
    priviusFrame = now;

    return false;
  }

  
  public void moveScurvePID(double distance) {
		double maxSpeed = 0.15;
		double lspeed = -MathUtil.clamp(scurveControllerl.calculate(-(m_Chassis.getLeftEncoderPosition() - scalibratedStartLeft), distance),-maxSpeed,maxSpeed);
		double rspeed = MathUtil.clamp(scurveControllerr.calculate(m_Chassis.getRightEncoderPosition() - scalibratedStartRight, distance),-maxSpeed,maxSpeed);

		SmartDashboard.putNumber("leftDriveError", scurveControllerl.getPositionError());
		SmartDashboard.putNumber("RightDriveError", scurveControllerr.getPositionError());
    
		m_Chassis.setLeft(lspeed);
		m_Chassis.setRight(rspeed);
	}


}
