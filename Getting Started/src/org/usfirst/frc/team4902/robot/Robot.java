package org.usfirst.frc.team4902.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends IterativeRobot {
	
	CameraServer server;
	Joystick stick;
	RobotDrive myRobot;
	
	Encoder liftEncoder;
	final int liftChannelA = 5;
	final int liftChannelB = 6;
	
	final int frontLeftChannel = 0;
	final int frontRightChannel = 0;
	final int rearLeftChannel = 0;
	final int rearRightChannel = 0;
	
	final int joystickLeftX = 0;
	final int joystickLeftY = 1;
	final int joystickRightX  = 4;
	
	final int joystickChannel	= 0;
	
    public void robotInit() {
    	
        server = CameraServer.getInstance();
        server.setQuality(20);
        server.startAutomaticCapture("cam0");
        
        myRobot = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
        
        //robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
    	//robotDrive.setInvertedMotor(MotorType.kRearLeft, true);		// might need to change
        
        stick = new Joystick(joystickChannel);
        
        stick.setAxisChannel(Joystick.AxisType.kX, joystickLeftX);
        stick.setAxisChannel(Joystick.AxisType.kY, joystickLeftY);
        stick.setAxisChannel(Joystick.AxisType.kZ, joystickRightX);
    }

    public void autonomousPeriodic() {
    	
    }

    public void teleopPeriodic() {
    	
    	myRobot.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getZ(), 0);
    	
    	System.out.println(liftEncoder.getDirection() + ", " + liftEncoder.getDistance());
    	
    	Timer.delay(0.005);
    }
    
}
