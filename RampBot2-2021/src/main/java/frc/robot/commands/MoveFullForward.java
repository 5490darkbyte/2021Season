package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.io.FileWriter;


/**
 *
 */
public class MoveFullForward extends Command {

    private static int c = 0;

    private double speed;
    private static double totalS = 0;

    private static double[] testCycles = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    int cycleInd = 0;
    
    private static double totalV = 0;

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
        if (cycleInd >= testCycles.length)
        {
            Robot.m_Chassis.StopMotors();
        }
        
        else if (c<60 || c%4 != 0)
        {
            Robot.m_Chassis.moveLeftForward(testCycles[cycleInd]);
        }
        else if (c<500)
        {
            Robot.m_Chassis.moveLeftForward(testCycles[cycleInd]);
            
            totalS += Robot.m_Chassis.getLeftVelocity();
            totalV += Robot.m_Chassis.getBusVoltage();
      
        }
        else{
            double aveS = totalS / (110.0);
            double aveV = totalV / (110.0);
            
            SmartDashboard.putNumber("Cycle"+testCycles[cycleInd]+"Vel", aveS);
            SmartDashboard.putNumber("Cycle"+testCycles[cycleInd]+"Volt", aveV);

            totalS = 0;
            totalV = 0;
            c = 0;
            cycleInd ++;
        }
        
        
            // Can't get arrays to show on the SmartDashboard because it was never implemented
            // SmartDashboard.setDefaultNumberArray("test", speeds);
            // SmartDashboard.putNumberArray("speeds", speeds);
            // SmartDashboard.putStringArray("test", new String[4]);
        
        /*
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
            // currently 600
            final int CDIVIDE = 600;

            // number of speed2 velocities recorded for each currVolt value
            // currently 120
                // example: 120 velocities each are recorded for currVolts = 3, 4, 5, etc.
            final int VELNUM = CDIVIDE / INTERVAL;

        if (c <= 5000)
        {
            // currVolts gives gradually increasing number of volts from 3 - 11
                // currVolts = f(c) = c/600 + 3, a double rounded to nearest int
            double currVolts = (int) (c / 600) + 3.0;
            // moves robot forward depending on currVolts
            Robot.m_Chassis.moveForward(currVolts/12);
            
            if (c%5 == 0)
            {
                SmartDashboard.putNumber("Curr Left Motor Speed", Robot.m_Chassis.getLeftVelocity());
                speeds2[c/5] = Robot.m_Chassis.getLeftVelocity();
                voltages[c/5] = Robot.m_Chassis.getBusVoltage();
            }

            if ((c != 0) && (c % 600 == 0))
            {
                double totalS = 0;
                double totalV = 0;

                for (int i = c/5 - 120; i < c/5; i++) {
                    totalS += speeds2[i];
                    totalV += voltages[i];
                }

                double aveS = totalS /120.0;
                double aveV = totalV/120.0;

                String volts = currVolts+"";
                SmartDashboard.putString(volts, ""+"Average Speed: "+aveS+", Average Volts: "+aveV);

            }
        }
        */
        
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
            
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
