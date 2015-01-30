package org.usfirst.frc.team4902.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends IterativeRobot {
        
        Joystick stick;
        RobotDrive myRobot;
        Encoder liftEncoder;
        
        final int liftChannelA = 5;
        final int liftChannelB = 6;
        
        final int frontLeftChannel = 0;
        final int frontRightChannel = 3;
        final int rearLeftChannel = 2;
        final int rearRightChannel = 1;
        
        final int joystickLeftX = 0;
        final int joystickLeftY = 1;
        final int joystickRightX  = 4; //xBox controller = 4
        
        final int joystickChannel       = 1;
        
        private Talon frontLeftMotor;
        private Talon frontRightMotor;
        private Talon rearLeftMotor;
        private Talon rearRightMotor;
        
        final double speed = 0.2;
        final double STOP = 0.0;
        final double reverse = -1.0;
        final double delay = 1;
        
    public void robotInit() {
       
        
        frontLeftMotor = new Talon(frontLeftChannel);
        frontRightMotor = new Talon(frontRightChannel);
        rearLeftMotor = new Talon(rearLeftChannel);
        rearRightMotor = new Talon(rearRightChannel);
        
        myRobot = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        
        myRobot.setInvertedMotor(MotorType.kFrontRight, false);
        myRobot.setInvertedMotor(MotorType.kFrontLeft, true);
        myRobot.setInvertedMotor(MotorType.kRearRight, false);
        myRobot.setInvertedMotor(MotorType.kRearLeft, true);
        
        stick = new Joystick(joystickChannel);
        
        stick.setAxisChannel(Joystick.AxisType.kX, joystickLeftX);
        stick.setAxisChannel(Joystick.AxisType.kY, joystickLeftY);
        stick.setAxisChannel(Joystick.AxisType.kZ, joystickRightX);
    }

    public void autonomousPeriodic() {
        myRobot.mecanumDrive_Cartesian(0, 0.2, 0, 0);
        
    }

    public void teleopPeriodic() {
        
        System.out.println(stick.getX() + ", " + stick.getY());
        
        myRobot.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getZ(), 0);
        
        //System.out.println(liftEncoder.getDirection() + ", " + liftEncoder.getDistance());
        
        Timer.delay(0.005);
    }
    
    public void testPeriodic() {        
        
        test3();
        
    }
    
    public void test1(){
        
        frontLeftMotor.set(speed);
        Timer.delay(delay);
        frontLeftMotor.set(STOP);
        
        frontRightMotor.set(speed);
        Timer.delay(delay);
        frontRightMotor.set(STOP);
        
        rearRightMotor.set(speed);
        Timer.delay(delay);
        rearRightMotor.set(STOP);
        
        rearLeftMotor.set(speed);
        Timer.delay(delay);
        rearLeftMotor.set(STOP);
    
    }
    
    public void test2(){
    
        frontLeftMotor.set(speed);
        Timer.delay(1);
        frontLeftMotor.set(STOP);
        Timer.delay(1);
        frontLeftMotor.set(reverse);
        Timer.delay(1);
        frontLeftMotor.set(STOP);
        
        Timer.delay(5);
        
        frontRightMotor.set(speed);
        Timer.delay(1);
        frontRightMotor.set(STOP);
        Timer.delay(1);
        frontRightMotor.set(reverse);
        Timer.delay(1);
        frontRightMotor.set(STOP);
        
        Timer.delay(5);
        
        rearRightMotor.set(speed);
        Timer.delay(1);
        rearRightMotor.set(STOP);
        Timer.delay(1);
        rearRightMotor.set(reverse);
        Timer.delay(1);
        rearRightMotor.set(STOP);
        
        Timer.delay(5);
        
        rearLeftMotor.set(speed);
        Timer.delay(1);
        rearLeftMotor.set(STOP);
        Timer.delay(1);
        rearLeftMotor.set(reverse);
        Timer.delay(1);
        rearLeftMotor.set(STOP);
        
        Timer.delay(5);
        
    }
    
    public void test3() {
    	frontLeftMotor.set(speed);
    	frontRightMotor.set(speed);
    	rearLeftMotor.set(speed);
    	rearRightMotor.set(speed);
    }
    
}
