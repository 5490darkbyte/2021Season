// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Conveyor;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/** An example command that uses an example subsystem. */
public class MoveConveyor extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //Constructor for the command
  public MoveConveyor() {
    requires(Robot.m_Conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_Conveyor.LiftBall(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    Robot.m_Conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
