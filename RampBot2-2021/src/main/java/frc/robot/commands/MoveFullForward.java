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
    private static double[] speeds2 = new double[1000];

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
        SmartDashboard.putNumber("moveFullForward c: ", c);
        c++;

        // Commented out old version (use control + / to uncomment).

        // if (c<=500)
        // {
        //     Robot.m_Chassis.moveForward(speed);
            
        //     if (c%5 == 0)
        //     {
        //         speeds[c/5] = Robot.m_Chassis.getLeftVelocity();
        //         total += speeds[c/5];
        //     }
      
        //     double ave = total / (c/5);
        //     SmartDashboard.putNumber("Ave Left Spd", ave);

        //     // Can't get arrays to show on the SmartDashboard for some reason?
        //     // SmartDashboard.setDefaultNumberArray("test", speeds);
        //     // SmartDashboard.putNumberArray("speeds", speeds);
        //     // SmartDashboard.putStringArray("test", new String[4]);
        // }
        
        // Uses speeds2 instead of speeds; speeds2 has 1000 values (5000 / 5 = 1000)

            // Variables for keeping track of velocity/c
            // NOT BEING USED because it could be confusing

            // currently 1000
            final int LENG = speeds2.length;

            // when c % INTERVAL == 0, velocity is added to speeds2
            final int INTERVAL = 5;

            // max value of c
            // currently 5000
            final int MAXC = LENG * INTERVAL;

            // when c % CDIVIDE == 0, a new average is calculated and displayed on SmartDashboard
            // currently 500
            final int CDIVIDE = MAXC / INTERVAL;

        if (c <= 5000)
        {
            // currVolts gives gradually increasing number of volts from 1 - 10
                // depending on current value of c
                // +1 because otherwise it'd go 0-9 volts
            // moves robot forward depending on currVolts
            double currVolts = (int) (c / 500) + 1.0;
            Robot.m_Chassis.moveForward(currVolts/12);
            
            if (c%5 == 0)
            {
                speeds2[c/5] = Robot.m_Chassis.getLeftVelocity();
            }

            if ((c != 0) && (c % 500 == 0))
            {
                int total2 = 0;

                for (int i = c/5 - 100; i < c/5; i++) {
                    total2 += speeds2[i];
                }

                double ave = total2 / 100;
                String volts = currVolts+"";
                SmartDashboard.putNumber(volts, ave);

            }
        }

        
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
