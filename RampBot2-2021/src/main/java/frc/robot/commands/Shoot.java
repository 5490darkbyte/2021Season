// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  double cycCount = 0.1;
  int c = 0;
  double sumSpeeds = 0;
  double sumVoltages = 0;
  double sumPositions = 0;
  double sumPWVelocities = 0;

  boolean isForward;
  //Constructor for the command
  public Shoot(boolean isForward) {
    this.isForward = isForward;
    requires(Robot.m_Shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (cycCount <= 1)
    {
      Robot.m_Shoot.spinMotors(cycCount);
      speedTest();
    }
    else{
      Robot.m_Shoot.stop();
    }
    

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    Robot.m_Shoot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void speedTest()
  {
    if (c>=100 && c % 4 == 0)
    {
      sumSpeeds += Robot.m_Shoot.getLeftVelocity() * (75.0/512.0);
      sumVoltages += Robot.m_Shoot.getBusVoltage();
      //sumPositions += Robot.m_Shoot.getPWPosition();
      //sumPWVelocities += Robot.m_Shoot.getPWVelocity();
      SmartDashboard.putNumber("Added nums to total. Current cycle: ", cycCount);
    }
    c++;
    SmartDashboard.putNumber("Counter c: ", c);
    if (c==499)
    {
      SmartDashboard.putString("Cycle"+cycCount+" sumSpeeds: ", sumSpeeds/100+"");
      SmartDashboard.putString("Cycle"+cycCount+" sumVoltages: ", sumVoltages/100+"");
      //SmartDashboard.putString("Cycle"+cycCount+" sumPositions: ", sumPositions/400+"");
      //SmartDashboard.putString("Cycle"+cycCount+" sumPWVelocities: ", sumPWVelocities/400+"");

      c = 0;
      sumSpeeds = 0;
      sumVoltages = 0;
      //sumPositions = 0;
      //sumPWVelocities = 0;
      cycCount+=.1;
    }


  }
}
