package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.io.FileWriter;
import java.io.IOException;

/**
 *
 */
public class Move1SideForward extends Command {


    //private static int c = 0;
    private double speed;
    // static double total = 0;
    //private static double[] speeds = new double[10];

    public Move1SideForward() {
        requires(Robot.m_Chassis);
        speed = 0.2;
        
    }
    public Move1SideForward(double mySpeed) {
        requires(Robot.m_Chassis);
        speed = mySpeed;
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.m_Chassis.move1Side(speed, false);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.m_Chassis.StopMotors();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
