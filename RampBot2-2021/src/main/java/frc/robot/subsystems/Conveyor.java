package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import com.ctre.phoenix.motorcontrol.can.*;


public class Conveyor extends Subsystem
{
    // Conveyor belt motors and its settings
    private WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.conveyor);



    public Conveyor()
    {
        // Configuration for the motor
        // We are using a VexPro CIM motor

        // Configuration for the motor being used
    motor.configContinuousCurrentLimit(MotorConfigs.vexContinousCurrentLimit, 0);
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

    public void LiftBall(double speed)
    {
        speed = 0.1; // Set to a low speed starting off
        motor.set(speed);
    }

    public void stop()
    {
        motor.set(0);
    }
}
