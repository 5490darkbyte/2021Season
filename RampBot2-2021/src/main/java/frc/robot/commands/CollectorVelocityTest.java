// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Dr. Lawlis wanted a velocity test for the Collector
// So I merged MoveCollectorForward and MoveFullForward

package frc.robot.commands;

import frc.robot.subsystems.Collector;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class CollectorVelocityTest extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private static int c = 0;

  private double speed;
  private static double totalS = 0;

  private static double[] testCycles = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  int cycleInd = 0;
  
  private static double totalV = 0;

  //Constructor for the command
  public CollectorVelocityTest() {
    requires(Robot.m_Collector);
 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("moveFullForward c: ", c);
    c++;

    // Commented out old version (use control + / to uncomment).
    if (cycleInd >= testCycles.length)
    {
        Robot.m_Collector.stop();
    }
    
    else if (c<60 || c%4 != 0)
    {
        Robot.m_Collector.spinMotor(testCycles[cycleInd]);
    }
    else if (c<500)
    {
        Robot.m_Collector.spinMotor(testCycles[cycleInd]);
        
        //totalS += Robot.m_Collector.getLeftVelocity();
        // totalV += Robot.m_Collector.getBusVoltage();
    
    }
    else {
        double aveS = totalS / (110.0);
        double aveV = totalV / (110.0);
        
        SmartDashboard.putNumber("Cycle"+testCycles[cycleInd]+"Vel", aveS);
        SmartDashboard.putNumber("Cycle"+testCycles[cycleInd]+"Volt", aveV);

        totalS = 0;
        totalV = 0;
        c = 0;
        cycleInd ++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    // Robot.m_Collector.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
