package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.*;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  private WPI_TalonSRX testMotor = new WPI_TalonSRX(RobotMap.mtrLift);

  private Encoder m_ShootEncoder = new Encoder(RobotMap.liftEncoderA, RobotMap.liftEncoderB);
  
  public Shooter() {
    
    // Configuration for the motor being used
    testMotor.configContinuousCurrentLimit(40, 0);
		testMotor.configPeakCurrentLimit(55, 0);
		testMotor.configPeakCurrentDuration(100, 0);
		testMotor.enableCurrentLimit(true);	
    testMotor.configOpenloopRamp(0, 0);
    
    // Configuration for the encoders
    //m_LiftEncoder.setDistancePerPulse(mm_per_turn / pulses_per_revolution);
    
      
    addChild("Motor im testing", testMotor);
    addChild("Encoder being used", m_ShootEncoder);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void SpinMotors(int time)
  {
      testMotor.set(0.1);
  }
  public void stop()
  {
    testMotor.set(0);
  }
}