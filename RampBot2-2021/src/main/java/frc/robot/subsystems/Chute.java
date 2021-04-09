package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

public class Chute extends Subsystem
{
    // Chute belt motors and its settings
    private WPI_TalonSRX lowerMotor = new WPI_TalonSRX(RobotMap.lowerChute);
    
    private WPI_TalonSRX upperMotor = new WPI_TalonSRX(RobotMap.upperChute);

    private double mySpeed;

    public Chute()
    {
        // Configuration for the motor
        // We are using a VexPro CIM motor
    mySpeed = MotorConfigs.slowChuteSpeed;

    // Configuration for the lower motor being used
    lowerMotor.configContinuousCurrentLimit(MotorConfigs.vexContinousCurrentLimit, 0);
    lowerMotor.configPeakCurrentLimit(MotorConfigs.vexPeakCurrent, 0);
    lowerMotor.configPeakCurrentDuration(MotorConfigs.vexPeakDuration, 0);
    lowerMotor.enableCurrentLimit(true);	
    lowerMotor.configOpenloopRamp(0, 0);

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
        mySpeed = MotorConfigs.fastChuteSpeed;
    }
    public void switchToNormal()
    {
        mySpeed = MotorConfigs.slowChuteSpeed;
    }
    public double getMySpeed()
    {
        return mySpeed;
    }

    public void moveLower(double speed)
    {
        
        double maxSpeed = MotorConfigs.maxChuteSpeed;

        if (speed>maxSpeed) {
            speed = maxSpeed;
        }
        
        // Set to a low speed starting off
        lowerMotor.set(speed);
    }

    public void stopLower()
    {
        lowerMotor.set(0);
    }

    public void moveUpper(double speed)
    {
        double maxSpeed = MotorConfigs.maxChuteSpeed;
        if (speed>maxSpeed)
        {
            speed = maxSpeed;
        }
        upperMotor.set(speed);
    }
    public void stopUpper()
    {
        upperMotor.set(0);
    }

    public void moveFullChute(double speed)
    {
        double maxSpeed = MotorConfigs.maxChuteSpeed;
        if (speed>maxSpeed)
        {
            speed = maxSpeed;
        }

        lowerMotor.set(speed);
        upperMotor.set(speed);
    }
    public void stopAll()
    {
        lowerMotor.set(0);
        upperMotor.set(0);
    }

    
}
