package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.RobotMap;
import frc.robot.MotorConfigs;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.commands.CollectorCommand;

import com.ctre.phoenix.sensors.CANCoder;



public class Collector extends Subsystem
{
    WPI_TalonSRX motorWheels = new WPI_TalonSRX(RobotMap.collectorWheels);
    WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.collector);

    CANCoder encoder = new CANCoder(RobotMap.collectorEncoder);

    PIDController collectorController = new PIDController(0.005, 0.003, 0.00002);
    

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

        collectorController.setIntegratorRange(-05, 05);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void initDefaultCommand()
    {
        
        setDefaultCommand(new CollectorCommand());
    }

    public void rotateCollectorPIDTick(double angle) {
        
        if ( getEncoderPosition() > MotorConfigs.maxCollectorAngle + 10)
        {
            motor.set(0);
        }
        else {
            motor.set(MathUtil.clamp(collectorController.calculate(getEncoderPosition(), angle),-0.15,0.15));
        }
    }

    public void rotateCollectorPIDReset() {
        collectorController.reset();
        System.out.print("reset controller");
    }

    public void rotateCollector(Joystick xbox)
    {   
        double yAxis = xbox.getY();
        double minimumMove = 0.02;
        double speedrange = .05-minimumMove;
		double speed = (-speedrange*yAxis+1)/2;
        speed += minimumMove;

        rotateCollector(speed*xbox.getY()*0.25);
    }
    public void rotateCollector(double speed)
    {
        if (speed < 0 && motor.getSensorCollection().isRevLimitSwitchClosed())
        {   
            // SmartDashboard.putNumber("yAxis",yAxis);
            encoder.setPosition(0);
            motor.set(0);
        }
        else if (speed > 0 && getEncoderPosition() > MotorConfigs.maxCollectorAngle)
        {

            motor.set(0);
        }
        else
        {
            motor.set(speed);
        }
    }

    
    public void spinWheels()
    {
        motorWheels.set(0.25);
    }
    public void stopWheels()
    {
        motorWheels.set(0);
    }

    public double getEncoderPosition()
    {
        return encoder.getPosition();
    }
    public double getEncoderVelocit() {
        return encoder.getVelocity();
    }

    public boolean limitSeitchActive() {
        return motor.getSensorCollection().isRevLimitSwitchClosed();
    }
}
