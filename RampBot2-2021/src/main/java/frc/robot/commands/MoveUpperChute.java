// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.UpperChute;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class MoveUpperChute extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  //Constructor for the command
  public MoveUpperChute() {
    requires(Robot.m_UpperChute);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Robot.m_UpperChute.getMySpeed();
    Robot.m_UpperChute.moveUp(speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    Robot.m_UpperChute.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
