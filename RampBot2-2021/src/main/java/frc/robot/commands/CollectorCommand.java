package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CollectorCommand extends Command {
	
    static int c = 0;
    public CollectorCommand() {
    	requires(Robot.m_Collector);    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
        c++;
        Robot.m_Collector.rotateCollector(Robot.m_oi.getXbox());
        if (Robot.m_oi.getXbox().getX()>0.2)
        {
            Robot.m_Collector.spinWheels();
        }
        else
        {
            Robot.m_Collector.stopWheels();
        }
        SmartDashboard.putNumber("Encoder Position", Robot.m_Collector.getEncoderPosition());
        SmartDashboard.putNumber("Collect count", c);

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
    		end();
    }
}
