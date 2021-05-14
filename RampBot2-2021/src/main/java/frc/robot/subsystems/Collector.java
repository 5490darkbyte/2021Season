package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

import frc.robot.commands.CollectorCommand;

public class Collector extends Subsystem
{
    WPI_TalonSRX motorWheels = new WPI_TalonSRX(RobotMap.collectorWheels);
    WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.collector);

    //DigitalInput upLimitSwitch = new DigitalInput(RobotMap.mtrLift());

    public Collector()
    {
        motor.configContinuousCurrentLimit(MotorConfigs.vexContinuousCurrentLimit, 0);
		motor.configPeakCurrentLimit(MotorConfigs.vexPeakCurrent, 0);
		motor.configPeakCurrentDuration(MotorConfigs.vexPeakDuration, 0);
		motor.enableCurrentLimit(true);	
        motor.configOpenloopRamp(0, 0);

        motorWheels.configContinuousCurrentLimit(MotorConfigs.vexSmallContinuousCurrentLimit, 0);
		motorWheels.configPeakCurrentLimit(MotorConfigs.vexSmallPeakCurrent, 0);
		motorWheels.configPeakCurrentDuration(MotorConfigs.vexSmallPeakDuration, 0);
		motorWheels.enableCurrentLimit(true);	
        motorWheels.configOpenloopRamp(0, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void initDefaultCommand()
    {
        setDefaultCommand(new CollectorCommand());
    }

    public void rotateCollector(Joystick xbox)
    {   
        double minThrottle = 0.05;
        double moveSpeed = (xbox.getY()*0.3);
        if (moveSpeed >= minThrottle)
        {
            motor.set(moveSpeed);
        }
        else
        {
            motor.set(0);
        }
        
    }

    public void spinWheels()
    {
        motorWheels.set(0.1);
    }
    public void stopWheels()
    {
        motorWheels.set(0);
    }
}
