// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FullRetractCollectorPID extends Command {
  /** Creates a new FullRetractCollectorPID. */
  public FullRetractCollectorPID() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Robot.m_Collector);
  }

  private double setAngle = 0.0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_Collector.rotateCollectorPIDTick(setAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    Robot.m_Collector.rotateCollectorPIDReset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Robot.m_Collector.limitSeitchActive() || (Math.abs(Robot.m_Collector.getEncoderPosition() - setAngle) < 2) && Robot.m_Collector.getEncoderVelocit() == 0);
  }
}
