package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.*;

public class Conveyor extends Subsystem
{
    // Conveyor belt motors and its settings
    private WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.mtrRamp);

    private final int continuousMaxCurrent = 40;  // Set this to the boards max current (ie. 40)
    private final int maxCurrent = 60; // Because 1.2*40 = around 60 < found on website, go with 60
    private final int maxCurrentDuration = 100; // In milliseconds (keep at 100 for now)



    public Conveyor()
    {
        // Configuration for the motor
        // We are using a VexPro CIM motor

        // Configuration for the motor being used
    motor.configContinuousCurrentLimit(continuousMaxCurrent, 0);
    motor.configPeakCurrentLimit(maxCurrent, 0);
    motor.configPeakCurrentDuration(maxCurrentDuration, 0);
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
