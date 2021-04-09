package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import com.ctre.phoenix.motorcontrol.can.*;

public class Shooter extends Subsystem {
  /** Creates a new ExampleSubsystem. */
  
  private WPI_TalonSRX testMotor = new WPI_TalonSRX(RobotMap.shooter1);
  
  private WPI_TalonSRX rightShooter = new WPI_TalonSRX(RobotMap.shooter2);

  private SpeedControllerGroup m_shooterMotors = new SpeedControllerGroup(testMotor, rightShooter);

  
  
  public Shooter() {
    
    // Configuration for the motor being used
    testMotor.configContinuousCurrentLimit(MotorConfigs.redlineContinuousCurrentLimit, 0);
		testMotor.configPeakCurrentLimit(MotorConfigs.redlinePeakCurrent, 0);
		testMotor.configPeakCurrentDuration(MotorConfigs.redlinePeakDuration, 0);
		testMotor.enableCurrentLimit(true);	
    testMotor.configOpenloopRamp(0, 0);
    testMotor.setInverted(true);

    rightShooter.configContinuousCurrentLimit(MotorConfigs.redlineContinuousCurrentLimit, 0);
		rightShooter.configPeakCurrentLimit(MotorConfigs.redlinePeakCurrent, 0);
		rightShooter.configPeakCurrentDuration(MotorConfigs.redlinePeakDuration, 0);
		rightShooter.enableCurrentLimit(true);	
    rightShooter.configOpenloopRamp(0, 0);

    
    
    // Configuration for the encoders
    //m_LiftEncoder.setDistancePerPulse(mm_per_turn / pulses_per_revolution);
    
      
    addChild("Motor im testing", testMotor);
    



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

  public void SpinMotors(int time)
  {
      m_shooterMotors.set(MotorConfigs.shooterSpeed);
  }
  public void stop()
  {
    m_shooterMotors.set(0);
  }
}