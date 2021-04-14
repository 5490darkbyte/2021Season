// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.*;
// import com.ctre.phoenix.sensors.CANCoder;

// import frc.robot.MySpeedControllerGroup;

// import edu.wpi.first.wpilibj.*;
// import edu.wpi.first.wpilibj.command.PIDSubsystem;
// import frc.robot.RobotMap;

// public class ChassisPID2 extends PIDSubsystem { // This system extends PIDSubsystem

//     Victor motor = RobotMap.wristMotor;
//     AnalogInput pot = RobotMap.wristPot();

//     public Wrist() {
//             super("Wrist", 2.0, 0.0, 0.0);// The constructor passes a name for the subsystem and the P, I and D constants that are useed when computing the motor output
//             setAbsoluteTolerance(0.05);
//             getPIDController().setContinuous(false); //manipulating the raw internal PID Controller
//     }

//     public void initDefaultCommand() {
//     }

//     protected double returnPIDInput() {
//             return pot.getAverageVoltage(); // returns the sensor value that is providing the feedback for the system
//     }

//     protected void usePIDOutput(double output) {
//             motor.pidWrite(output); // this is where the computed output value fromthe PIDController is applied to the motor
//     }
// }