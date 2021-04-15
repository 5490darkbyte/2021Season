// package frc.robot.commands;

// import frc.robot.Robot;

// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.PIDCommand;

// /**
//  *
//  */
// public class MoveFullForward extends PIDCommand {

//     public static int a = 0;
//     private double speed;
//     public MoveFullForward() {
//         requires(Robot.m_Chassis);
//         speed = 1;
        
//     }
//     public MoveFullForward(double mySpeed) {
//         requires(Robot.m_Chassis);
//         speed = mySpeed;
        
//     }

//     // Called just before this Command runs the first time
//     protected void initialize() {
//     }

//     // Called repeatedly when this Command is scheduled to run
//     protected void execute() {
//         a++;
//         Robot.m_Chassis.moveForward(speed);
        
//     }

//     // Make this return true when this Command no longer needs to run execute()
//     protected boolean isFinished() {
//         return false;
//     }

//     // Called once after isFinished returns true
//     protected void end() {
//     		Robot.m_Chassis.StopMotors();
//     }

//     // Called when another command which requires one or more of the same
//     // subsystems is scheduled to run
//     protected void interrupted() {
//     }
// }
