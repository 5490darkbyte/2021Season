package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class MoveFullForward extends Command {

    private static int c = 0;
    private double speed;
    private static double total = 0;
    private static double[] speeds = new double[100];

    public MoveFullForward() {
        requires(Robot.m_Chassis);
        speed = 0.2;
        
    }
    public MoveFullForward(double mySpeed) {
        requires(Robot.m_Chassis);
        speed = mySpeed;
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        SmartDashboard.putNumber("C", c);
        c++;
        if (c<=500)
        {
            Robot.m_Chassis.moveForward(speed);
            
            
            if (c%5 == 0)
            {
                speeds[c/5] = Robot.m_Chassis.getLeftVelocity();
                total += speeds[c/5];
            }
      
            
            // for (double spd : speeds)
            // {
            //     total += spd;
            // }
            double ave = total / (c/5);
            SmartDashboard.putNumber("Ave Left Spd", ave);
            SmartDashboard.putNumberArray("speeds", speeds);
            SmartDashboard.putStringArray("test", new String[4]);
        }
        // if (c == 499) {
            
        // }
        


        
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
