package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Paddles extends Subsystem
{
    
    Servo leftServo = new Servo(0);
    Servo rightServo = new Servo(1);

    public Paddles()
    {
        //leftServo.set(60);
        //rightServo.set(60);
        //leftServo.setSpeed(0.2);
        //rightServo.setSpeed(0.2);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void initDefaultCommand()
    {
        
    }
    

    public void openPaddles(int degrees)
    {
        leftServo.setAngle(30);
        rightServo.setAngle(150);
    }

    public void closePaddles()
    {
        leftServo.setAngle(125);
        rightServo.setAngle(50);
    }
}
