package frc.robot;
public class MotorConfigs
{
    public static final int universalCurrentLimit = 39;
    public static final int universalPeakDuration = 100;
    // VEX Robotics CIM Motors
    public static final int vexContinuousCurrentLimit = universalCurrentLimit;
    public static final int vexPeakCurrent = 60;
    public static final int vexPeakDuration = universalPeakDuration;

    // VEX Robotics 775pro motor (smaller than VEX CIM motor)
    public static final int vexSmallContinuousCurrentLimit = 20;
    public static final int vexSmallPeakCurrent = 25;
    public static final int vexSmallPeakDuration = universalPeakDuration;

    // Andymark Redline Motors
    public static final int redlineContinuousCurrentLimit = universalCurrentLimit;
    public static final int redlinePeakCurrent = 55;
    public static final int redlinePeakDuration = universalPeakDuration;

    public static final double redlineLeftCruiseVelocity = 58617.36;
    public static final double redlineRightCruiseVelocity = 66768.0;

    public static final double redlineLeftAccel = 58617.36;
    public static final double redlineRightAccel = 66768.0;

    // Chute Configs
    public static final double maxChuteSpeed = 0.5;
    public static final double[] slowChuteSpeed = {-0.35, -0.25};
    public static final double[] fastChuteSpeed = {-0.5, -.375};

    // Shooter Configs
    // Best is 0.93
    public static final double shooterSpeed = 0.8;

    // in rpm
    public static final double shooterTargetVel = 16000; //8587;

    // Collections Configs
    public static final double maxCollectorAngle = 202.6;
}