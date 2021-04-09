package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.commands.LiftManualMove;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// this is for the Talon SRX version
//import edu.wpi.first.wpilibj.Talon;
//import com.revrobotics.*;


import frc.robot.RobotMap;
import frc.robot.commands.DriveRobot;

//import frc.robot.Point3D;





import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.AnalogInput;


//import com.analog.adis16448.frc.ADIS16448_IMU;
//import jdk.internal.jshell.tool.resources.l10n;

public class DifferentialDrive2 extends DifferentialDrive {
    
    public DifferentialDrive2(speedController leftMotor, speedController rightMotor, int ) {
        super(leftMotor, rightMotor);
    }

    @Override
    public void arcadeDrive() {

    }
}
