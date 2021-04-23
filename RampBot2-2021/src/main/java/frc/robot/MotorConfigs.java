package frc.robot;
public class MotorConfigs
{
    public static int universalCurrentLimit = 40;
    public static int universalPeakDuration = 100;
    // VEX Robotics CIM Motors
    public static int vexContinousCurrentLimit = universalCurrentLimit;
    public static int vexPeakCurrent = 60;
    public static int vexPeakDuration = universalPeakDuration;

    // Andymark Redline Motors
    public static int redlineContinuousCurrentLimit = universalCurrentLimit;
    public static int redlinePeakCurrent = 55;
    public static int redlinePeakDuration = universalPeakDuration;

    public static double redlineLeftCruiseVelocity = 58617.36;
    public static double redlineRightCruiseVelocity = 66768.0;

    public static double redlineLeftAccel = 58617.36;
    public static double redlineRightAccel = 66768.0;

    // Chute Configs
    public static double maxChuteSpeed = 0.5;
    public static double[] slowChuteSpeed = {-0.35, -0.25};
    public static double[] fastChuteSpeed = {-0.5, -.375};

    // Shooter Configs
    // Best is 0.93
    public static double shooterSpeed = 0.5;
}