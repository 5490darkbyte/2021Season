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
}