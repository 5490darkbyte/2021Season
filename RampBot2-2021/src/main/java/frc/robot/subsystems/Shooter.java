package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import java.util.ArrayList;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.CANCoder;



public class Shooter extends Subsystem {
  /** Creates a new ExampleSubsystem. */
  
  private WPI_TalonSRX leftShooter = new WPI_TalonSRX(RobotMap.shooter1);
  private WPI_TalonSRX rightShooter = new WPI_TalonSRX(RobotMap.shooter2);
  private SpeedControllerGroup m_shooterMotors = new SpeedControllerGroup(leftShooter, rightShooter);
  

  private double roulingAVGLength = 100;
  private ArrayList<Double> roulingLeftValues = new ArrayList<Double>();
  private ArrayList<Double> roulingRightValues = new ArrayList<Double>();


  // private CANCoder leftEncoder = new CANCoder(RobotMap.shooter1);
  // private CANCoder rightEncoder = new CANCoder(RobotMap.shooter2);

  //private PIDController leftController = new PIDController(0.01, 0, 0);
  //private PIDController rightController = new PIDController(0.01, 0, 0);
  
  
  public Shooter() {
    
    // Configuration for the motor being used
    leftShooter.configContinuousCurrentLimit(MotorConfigs.redlineContinuousCurrentLimit, 0);
		leftShooter.configPeakCurrentLimit(MotorConfigs.redlinePeakCurrent, 0);
		leftShooter.configPeakCurrentDuration(MotorConfigs.redlinePeakDuration, 0);
		leftShooter.enableCurrentLimit(true);	
    leftShooter.configOpenloopRamp(0, 0);
    leftShooter.setInverted(true); // invert left shooter motor

    //Motionmagic extra configurations
    leftShooter.configMotionCruiseVelocity(MotorConfigs.redlineLeftCruiseVelocity);
    leftShooter.configMotionAcceleration(MotorConfigs.redlineLeftAccel);

    rightShooter.configContinuousCurrentLimit(MotorConfigs.redlineContinuousCurrentLimit, 0);
		rightShooter.configPeakCurrentLimit(MotorConfigs.redlinePeakCurrent, 0);
		rightShooter.configPeakCurrentDuration(MotorConfigs.redlinePeakDuration, 0);
		rightShooter.enableCurrentLimit(true);	
    rightShooter.configOpenloopRamp(0, 0);
    //Additional Motionmagic configurations
    rightShooter.configMotionCruiseVelocity(MotorConfigs.redlineRightCruiseVelocity);
    leftShooter.configMotionAcceleration(MotorConfigs.redlineRightAccel);

    
    //PID and Encoder Settings

    leftShooter.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 150);
    leftShooter.selectProfileSlot(0, 0);

    // leftShooter.setSensorPhase(true);
//TODO: extract coeficients to constants
    leftShooter.config_kP(0, 0.015);
    leftShooter.config_kI(0, 0.00004);
    leftShooter.config_IntegralZone(0, 5000);
    leftShooter.config_kD(0, 0.3);
    leftShooter.config_kF(0, MotorConfigs.leftShooterKf);


    //rightShooter.configre
    
    rightShooter.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 150);
    rightShooter.selectProfileSlot(0, 0);

    rightShooter.config_kP(0, 0.015);
    rightShooter.config_kI(0, 0.00004);
    rightShooter.config_IntegralZone(0, 5000);
    rightShooter.config_kD(0, 0.3);
    rightShooter.config_kF(0, MotorConfigs.rightShooterKf);

    // Configuration for the encoders
    //m_LiftEncoder.setDistancePerPulse(mm_per_turn / pulses_per_revolution);
    
      
    addChild("Motor I'm testing", leftShooter);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double leftv = measuredUnitsTorpm(leftShooter.getSelectedSensorVelocity());
    double rightv = measuredUnitsTorpm(rightShooter.getSelectedSensorVelocity());

    SmartDashboard.putNumber("shooter Left",leftv);
    SmartDashboard.putNumber("shooter Right", rightv);
    
    roulingLeftValues.add(leftv);
    roulingRightValues.add(rightv);
           
    if (roulingLeftValues.size() > roulingAVGLength) {
      roulingLeftValues.remove(0);
    }

              
    if (roulingRightValues.size() > roulingAVGLength) {
      roulingRightValues.remove(0);
    }

    if (roulingLeftValues.size() != 0 && roulingRightValues.size() != 0) {
      

      SmartDashboard.putNumber("avg shooter Left",roulingLeftValues.stream().reduce(0.0, (subtotal, element) -> subtotal + element) / roulingLeftValues.size());
      
      // SmartDashboard.putNumber("shooter Left v", leftEncoder.getVelocity());

      SmartDashboard.putNumber("avg shooter Right", roulingRightValues.stream().reduce(0.0, (subtotal, element) -> subtotal + element) / roulingRightValues.size());

    }
    // // SmartDashboard.putNumber("shooter Left",measuredUnitsTorpm(leftShooter.getSensorCollection().getQuadratureVelocity()));

    // // SmartDashboard.putNumber("shooter Right", measuredUnitsTorpm(rightShooter.getSensorCollection().getQuadratureVelocity()));

  }


  double measuredUnitsTorpm(double measured) {
    return (measured / 4096) * 600;
  }
  
  double rpmtoMeasuredUnits(double rpm) {
    return (rpm / 600) * 4096;
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
        // old fixed power setting
        // m_shooterMotors.set(MotorConfigs.shooterSpeed);

        //calebration testing
        // m_shooterMotors.set(0.8);

        // pid setting
        leftShooter.set(ControlMode.Velocity, rpmtoMeasuredUnits(MotorConfigs.shooterTargetVel));
        rightShooter.set(ControlMode.Velocity, rpmtoMeasuredUnits(MotorConfigs.shooterTargetVel));
      }
      else
      {
        // m_shooterMotors.set(MotorConfigs.shooterSpeed*-1);
      }

  }
  public void spinMotors(double speed)
  {
    m_shooterMotors.set(speed);
    
  }

  public void pidSpinMotors(double speed) {

  }

  public void stop()
  {
    leftShooter.set(ControlMode.Velocity, 0);
    rightShooter.set(ControlMode.Velocity, 0);
    // m_shooterMotors.set(0);
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