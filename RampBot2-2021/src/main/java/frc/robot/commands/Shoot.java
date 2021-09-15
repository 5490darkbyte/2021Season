// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MotorConfigs;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  int c = 0;
  double sumSpeeds = 0;
  boolean isForward;
  //Constructor for the command
  public Shoot(boolean isForward) {
    this.isForward = isForward;
    requires(Robot.m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Robot.m_Shooter.spinMotors(true);

    // if (c>=200 && c%4==0 && c<1000)
    // {
    //   sumSpeeds += Robot.m_Shooter.getLeftVelocity();
    // }
    // else if (c==1000)
    // {
    //   SmartDashboard.putNumber("MaxSpeed", sumSpeeds/200);
    // }
    
    // c++;



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    Robot.m_Shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
