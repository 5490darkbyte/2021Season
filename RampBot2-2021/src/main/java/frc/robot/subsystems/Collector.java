package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

public class Collector extends Subsystem
{
    
    WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.collector);

    //DigitalInput upLimitSwitch = new DigitalInput(RobotMap.mtrLift());

    public Collector()
    {
        motor.configContinuousCurrentLimit(MotorConfigs.vexContinuousCurrentLimit, 0);
		motor.configPeakCurrentLimit(MotorConfigs.vexPeakCurrent, 0);
		motor.configPeakCurrentDuration(MotorConfigs.vexPeakDuration, 0);
		motor.enableCurrentLimit(true);	
        motor.configOpenloopRamp(0, 0);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void initDefaultCommand()
    {
        
    }

    public void spinMotor(double speed)
    {
        motor.set(speed);
    }
    public void stop()
    {
        motor.set(0);
    }
}
