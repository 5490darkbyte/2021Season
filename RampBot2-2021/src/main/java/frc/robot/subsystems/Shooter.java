package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.CANCoder;



public class Shooter extends Subsystem {
  /** Creates a new ExampleSubsystem. */
  
  private WPI_TalonSRX leftShooter = new WPI_TalonSRX(RobotMap.shooter1);
  private WPI_TalonSRX rightShooter = new WPI_TalonSRX(RobotMap.shooter2);


  private SpeedControllerGroup m_shooterMotors = new SpeedControllerGroup(leftShooter, rightShooter);

  private CANCoder leftEncoder = new CANCoder(RobotMap.shooter1);
  private CANCoder rightEncoder = new CANCoder(RobotMap.shooter2);

  
  public Shooter() {
    
    // Configuration for the motor being used
    leftShooter.configContinuousCurrentLimit(MotorConfigs.redlineContinuousCurrentLimit, 0);
		leftShooter.configPeakCurrentLimit(MotorConfigs.redlinePeakCurrent, 0);
		leftShooter.configPeakCurrentDuration(MotorConfigs.redlinePeakDuration, 0);
		leftShooter.enableCurrentLimit(true);	
    leftShooter.configOpenloopRamp(0, 0);
    leftShooter.setInverted(true);

    //Motionmatic extra configurations
    leftShooter.configMotionCruiseVelocity(MotorConfigs.redlineLeftCruiseVelocity);
    leftShooter.configMotionAcceleration(MotorConfigs.redlineLeftAccel);

    rightShooter.configContinuousCurrentLimit(MotorConfigs.redlineContinuousCurrentLimit, 0);
		rightShooter.configPeakCurrentLimit(MotorConfigs.redlinePeakCurrent, 0);
		rightShooter.configPeakCurrentDuration(MotorConfigs.redlinePeakDuration, 0);
		rightShooter.enableCurrentLimit(true);	
    rightShooter.configOpenloopRamp(0, 0);
    //Additional Motionmatic configurations
    rightShooter.configMotionCruiseVelocity(MotorConfigs.redlineRightCruiseVelocity);
    leftShooter.configMotionAcceleration(MotorConfigs.redlineRightAccel);
    
    
    // Configuration for the encoders
    //m_LiftEncoder.setDistancePerPulse(mm_per_turn / pulses_per_revolution);
    
      
    addChild("Motor I'm testing", leftShooter);
    



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initDefaultCommand()
  {
    // Nothing here right now.  Needed to compile the code
    
  }

  public void spinMotors(boolean isForward)
  {
      if (isForward)
      {
        m_shooterMotors.set(MotorConfigs.shooterSpeed);
      }
      else
      {
        m_shooterMotors.set(MotorConfigs.shooterSpeed*-1);
      }
  }
  public void spinMotors(double speed)
  {
    m_shooterMotors.set(speed);
  }
  public void stop()
  {
    m_shooterMotors.set(0);
  }

  public double getLeftVelocity() {
    return leftShooter.getSensorCollection().getQuadratureVelocity();
  }
  /*public double getPWPosition() {
    return leftShooter.getSensorCollection().getPulseWidthPosition();
  }
  public double getPWVelocity() {
    return leftShooter.getSensorCollection().getPulseWidthVelocity();
  }*/
  public double getBusVoltage() {
    return rightShooter.getBusVoltage();
  }
}