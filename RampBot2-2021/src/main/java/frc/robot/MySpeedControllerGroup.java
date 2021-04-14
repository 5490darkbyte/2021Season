package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class MySpeedControllerGroup extends SpeedControllerGroup
{
    private SpeedController[] controllers;
    private double throttle;
    public MySpeedControllerGroup(SpeedController[] speedControllers)
    {
        super(speedControllers);
        controllers = speedControllers;
        throttle = 1;
    }
    /* Alternate constructor
    public MySpeedControllerGroup(SpeedController speedController, SpeedController... speedControllers)
    {
        super(speedController, speedControllers);
        controllers = speedControllers;
        throttle = 1;
    }
    */

    public MySpeedControllerGroup(SpeedController[] speedControllers, double throttlePerc)
    {
        super(speedControllers);
        controllers = speedControllers;
        throttle = throttlePerc;
    }


    @Override
  public void set(double speed) {
    for (SpeedController speedController : controllers) {
      speedController.set(super.getInverted() ? -speed * throttle : speed*throttle);
    }
  }
}