package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

public class UpperChute extends Subsystem
{
    // Subsystem for the upper chute of the robot
    
    private WPI_TalonSRX upperMotor = new WPI_TalonSRX(RobotMap.upperChute);

    private double mySpeed;

    public UpperChute()
    {
        
    //Accesses 1st index of slowChute - 1st index is speed of upper chute 
    mySpeed = MotorConfigs.slowChuteSpeed[1];

    /* Configuration for the Upper motor being used
     Configuration for the motor
     We are using a VexPro CIM motor*/
    upperMotor.configContinuousCurrentLimit(MotorConfigs.vexContinousCurrentLimit, 0);
    upperMotor.configPeakCurrentLimit(MotorConfigs.vexPeakCurrent, 0);
    upperMotor.configPeakCurrentDuration(MotorConfigs.vexPeakDuration, 0);
    upperMotor.enableCurrentLimit(true);	
    upperMotor.configOpenloopRamp(0, 0);


    
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
        
        mySpeed = MotorConfigs.fastChuteSpeed[1];
        
    }
    public void switchToNormal()
    {
        mySpeed = MotorConfigs.slowChuteSpeed[1];
    }
    public double getMySpeed()
    {
        return mySpeed;
    }


    public void moveUp(double speed)
    {
        upperMotor.set(speed);
    }
    public void moveDown(double speed)
    {
        upperMotor.set(-1*speed);
    }
    public void stop()
    {
        upperMotor.set(0);
    }
    
}
