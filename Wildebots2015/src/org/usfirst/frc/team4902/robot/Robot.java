package org.usfirst.frc.team4902.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Gyro;

import java.util.*;

public class Robot extends IterativeRobot {
	
	//http://wpilib.screenstepslive.com/s/3120/m/7912/l/85770-measuring-rotation-of-a-wheel-or-other-shaft-using-encoders
	// encoder instructions
	
	int counter = 0;
	double lowest1 = 0;
	double lowest2 = 0;
	
	
        Joystick stick;
        RobotDrive myRobot;
        Encoder liftEncoder, wheel1Encoder, wheel2Encoder, wheel3Encoder, wheel4Encoder;
        Gyro gyro;
        PIDController pidController;
        PIDOutput pidOutput1, pidOutput2, pidOutput3, pidOutput4;
        
        final int liftChannelA = 5;
        final int liftChannelB = 6;
        
        final int frontLeftChannel = 0;
        final int frontRightChannel = 3;
        final int rearLeftChannel = 2;
        final int rearRightChannel = 1;
        
        final int liftChannel = 4;
        
        final int joystickLeftX = 0;
        final int joystickLeftY = 1;
        final int joystickRightX  = 4; //xBox controller = 4

        
        final int joystickChannel       = 0;
        final int moduleNum = 0;
        final double triggerThreshold = 0.1;
        final double joystickZeroThreshold = 0.15;
        
        private Talon frontLeftMotor;
        private Talon frontRightMotor;
        private Talon rearLeftMotor;
        private Talon rearRightMotor;
        
        private Talon liftMotor;
        private DigitalInput topLimit, bottomLimit;
        
        private Compressor airCompressor;
        private Solenoid s1, s2;
        
        final double speed = 0.2;
        final double STOP = 0.0;
        final double reverse = -1.0;
        final double delay = 1;
        
        final int rButton = 6;
        final int lButton = 5;
        final int rTrigger = 3;
        final int lTrigger = 2;
        
        private double offset;
        
        private PIDOutput pidOutput;     
        
        private boolean turning, startedCompressor;
        
        
    public class PrintPIDOutput implements PIDOutput{    
    	
    	 public void pidWrite(double output){
    		 
    		 //System.out.println(output);
    	 }
    	
    }
    public class OffsetOutput implements PIDOutput{
    	public void pidWrite (double output) {
    		//System.out.println(output);
    		offset = output;
    	}
    }
        
    public void robotInit() {
       
        
    	
        frontLeftMotor = new Talon(frontLeftChannel);
        frontRightMotor = new Talon(frontRightChannel);
        rearLeftMotor = new Talon(rearLeftChannel);
        rearRightMotor = new Talon(rearRightChannel);
        
        liftMotor = new Talon(liftChannel);
        
        myRobot = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        
        myRobot.setInvertedMotor(MotorType.kFrontRight, false);
        myRobot.setInvertedMotor(MotorType.kFrontLeft, true);
        myRobot.setInvertedMotor(MotorType.kRearRight, false);
        myRobot.setInvertedMotor(MotorType.kRearLeft, true);
        
        stick = new Joystick(joystickChannel);
        
        stick.setAxisChannel(Joystick.AxisType.kX, joystickLeftX);
        stick.setAxisChannel(Joystick.AxisType.kY, joystickLeftY);
        stick.setAxisChannel(Joystick.AxisType.kZ, joystickRightX);
        
        gyro = new Gyro(0);
    	gyro.initGyro();
    	gyro.reset();
        
        wheel1Encoder = new Encoder(3, 4, true, EncodingType.k4X);
        wheel1Encoder.setMaxPeriod(.1);
        wheel1Encoder.setMinRate(10);
        wheel1Encoder.setDistancePerPulse(5);
        wheel1Encoder.setReverseDirection(true);
        wheel1Encoder.setSamplesToAverage(7);
        
        pidOutput = new OffsetOutput();
        pidController = new PIDController(gyro.getAngle(), 0, 0, gyro, pidOutput);
        
        turning = false;
        startedCompressor = false;
        
        airCompressor = new Compressor(moduleNum);  //Digtial I/O,Relay\
        
        s1 = new Solenoid(moduleNum,2);                        // Solenoid port
        s2 = new Solenoid(moduleNum,3);
        
        topLimit = new DigitalInput(0);
        bottomLimit = new DigitalInput(1); //something is taking input 1 and 2
        
    }
    
    public void autonomousPeriodic() {
        myRobot.mecanumDrive_Cartesian(0, 0.25, 0, 0);
        
    }

    public double joystickZeroed(double input) {
    	if(Math.abs(input) <= joystickZeroThreshold) {
    		return 0;
    	}
    	return input;
    }
    
    /**
     * Checks whether the robot is turning and resets the gyro after turning
     * @param input
     */
    public void turning(double input){
    	if(Math.abs(input) > joystickZeroThreshold){
    		pidController.disable();
    		offset = 0;
    		turning = true;
    	}
    	if((turning == true)&&(Math.abs(input) < joystickZeroThreshold)){
    		pidController.setSetpoint(gyro.getAngle());
    		pidController.enable();
    		turning = false;
    	}
    }
    
    /**
     * Initialize for teleop
     */
    public void teleopInit(){
        System.out.println("created the pid Controller");
        pidController.enable();
    }
    
    /**
     * Loop for the teleop phase
     */
    public void teleopPeriodic() {
        //System.out.println(stick.getX() + ", " + stick.getY());
        double inputX = joystickZeroed(stick.getX());
        double inputY = joystickZeroed(stick.getY());
        double inputZ = joystickZeroed(stick.getZ());
        
        turning(inputZ);
        
        manualLift();
        
        myRobot.mecanumDrive_Cartesian(-inputX, -inputY, -0.35*inputZ+offset, 0);
                
        Timer.delay(0.005);
    }
    
    public void disabledInit() {
    	if(pidController != null) {
    		gyro.reset();
    		pidController.disable();
    	}
    }
    
    @Override
    public void testInit() {
        pidController.enable();
    }
    
    public void testPeriodic() {     
    	
    	if(startedCompressor == false){
    		startCompressor();
    	}
    	else{
    		startedCompressor = true;
    	}
    	
        double inputX = 0;
        double inputY = -0.0;
        double inputZ = 0;
        
        myRobot.mecanumDrive_Cartesian(-inputX, -inputY, -0.35*inputZ-offset*0, 0);
        
        Timer.delay(0.005);
        
    	//lift();        
    }
    
    public void test2(){
        frontLeftMotor.set(speed);
        frontRightMotor.set(speed*-1);
        rearRightMotor.set(speed*-1);
        rearLeftMotor.set(speed);
    }
    
    public void test3() {
    	
    	double[] array = new double[4];
    	
    	wheel1Encoder.startLiveWindowMode();
    	Timer.delay(1);
    	
    	array[0] = wheel1Encoder.getDistance();
    	lowest1 = wheel1Encoder.getDistance();
    	wheel1Encoder.getDirection();
    	
    	wheel1Encoder.stopLiveWindowMode();
    	
    	lowest1 = wheel1Encoder.getDistance();
    	
    	lowest2 = lowest1;
    	
    }
    
    public void manualLift() {
    	
    	if(stick.getRawAxis(rTrigger) > triggerThreshold){
    		if(topLimit.get()){
    			liftMotor.set(STOP);
    		}
    		else{
    			liftMotor.set(stick.getRawAxis(rTrigger));
    		}
    	}
    	else if (stick.getRawAxis(lTrigger) > triggerThreshold){
    		if(bottomLimit.get()){
    			liftMotor.set(STOP);
    		}
    		else{
    			liftMotor.set(-(stick.getRawAxis(lTrigger)));
    		}
    	}
    	else{
    		liftMotor.set(STOP);
    	}
    	
    	if(stick.getRawButton(rButton)) {
    		closeGripper();
    	}
    	else if (stick.getRawButton(lButton)) {
    		openGripper();
    	}
    }
    
    public void closeGripper(){
		s1.set(true);
		s2.set(false);
    }
    
    public void openGripper(){
    	s1.set(false);
    	s2.set(true);
    }
    
    public void lift() {
    	boolean applyBrake = true;
    	boolean liftUp = stick.getRawButton(1);
    	boolean liftDown = stick.getRawButton(2); //Not sure which buttons these are on the controller
    	
    	if(applyBrake == true) {
    		s1.set(true);
    	}
    	else if(applyBrake == false) {
    		s1.set(false);
    	}
    	
    	if(liftUp == true) {
    		applyBrake = false;
    		liftMotor.set(speed);
            liftMotor.set(STOP);
    	}
    	else {
    		applyBrake = true;
    	}
    	
    	if(liftDown == true) {
    		applyBrake = false;
    		liftMotor.set(reverse);
            liftMotor.set(STOP);
    	}
    	else {
    		applyBrake = true;
    	}
    }
    
    public void startCompressor(){
        airCompressor.start();
    }
    
    public void checkSwitches() {
    	
    }
}
