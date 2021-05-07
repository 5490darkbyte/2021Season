package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

public class LowerChute extends Subsystem
{
    // Chute belt motors and its settings
    private WPI_TalonSRX lowerMotor = new WPI_TalonSRX(RobotMap.lowerChute);
    

    private double mySpeed;

    public LowerChute()
    {
        // Configuration for the motor
        // We are using a VexPro CIM motor
        mySpeed = MotorConfigs.slowChuteSpeed[0];

        // Configuration for the lower motor being used
        lowerMotor.configContinuousCurrentLimit(MotorConfigs.vexContinuousCurrentLimit, 0);
        lowerMotor.configPeakCurrentLimit(MotorConfigs.vexPeakCurrent, 0);
        lowerMotor.configPeakCurrentDuration(MotorConfigs.vexPeakDuration, 0);
        lowerMotor.enableCurrentLimit(true);	
        lowerMotor.configOpenloopRamp(0, 0);
    
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void initDefaultCommand()
    {
        
    }

    public void switchToFast()
    {
        
        mySpeed = MotorConfigs.fastChuteSpeed[0];
        
    }
    public void switchToNormal()
    {
        mySpeed = MotorConfigs.slowChuteSpeed[0];
    }
    public double getMySpeed()
    {
        return mySpeed;
    }

    public void moveUp(double speed)
    {
        
        // Set to a low speed starting off
        lowerMotor.set(speed);
    }

    public void stop()
    {
        lowerMotor.set(0);
    }

    
}
