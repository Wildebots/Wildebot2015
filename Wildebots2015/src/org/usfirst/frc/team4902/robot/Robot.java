package org.usfirst.frc.team4902.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
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
        PIDOutput pidOutput;
        
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
        final double joystickZeroThreshold = 0.15;
        
        final int joystickChannel       = 0;
        
        private Talon frontLeftMotor;
        private Talon frontRightMotor;
        private Talon rearLeftMotor;
        private Talon rearRightMotor;
        
        private Talon liftMotor;
        
        private Compressor airCompressor;
        private Solenoid s1;
        
        final double speed = 0.2;
        final double STOP = 0.0;
        final double reverse = -1.0;
        final double delay = 1;
        
        private boolean isAngleZero = true;
        private double  PIDTime = 0;
        
        
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
        
//        liftEncoder = new Encoder(2, 3, true, EncodingType.k4X);
//        liftEncoder.setMaxPeriod(.1);
//        liftEncoder.setMinRate(10);
//        liftEncoder.setDistancePerPulse(5);
//        liftEncoder.setReverseDirection(true);
//        liftEncoder.setSamplesToAverage(7);
        
        wheel1Encoder = new Encoder(1, 2, true, EncodingType.k4X);
        wheel1Encoder.setMaxPeriod(.1);
        wheel1Encoder.setMinRate(10);
        wheel1Encoder.setDistancePerPulse(5);
        wheel1Encoder.setReverseDirection(true);
        wheel1Encoder.setSamplesToAverage(7);
        
//        wheel2Encoder = new Encoder(2, 3, true, EncodingType.k4X);
//        wheel2Encoder.setMaxPeriod(.1);
//        wheel2Encoder.setMinRate(10);
//        wheel2Encoder.setDistancePerPulse(5);
//        wheel2Encoder.setReverseDirection(true);
//        wheel2Encoder.setSamplesToAverage(7);
        
//        wheel3Encoder = new Encoder(2, 3, true, EncodingType.k4X);
//        wheel3Encoder.setMaxPeriod(.1);
//        wheel3Encoder.setMinRate(10);
//        wheel3Encoder.setDistancePerPulse(5);
//        wheel3Encoder.setReverseDirection(true);
//        wheel3Encoder.setSamplesToAverage(7);
        
//        wheel4Encoder = new Encoder(2, 3, true, EncodingType.k4X);
//        wheel4Encoder.setMaxPeriod(.1);
//        wheel4Encoder.setMinRate(10);
//        wheel4Encoder.setDistancePerPulse(5);
//        wheel4Encoder.setReverseDirection(true);
//        wheel4Encoder.setSamplesToAverage(7);
        
        //p is the gyro angle
        //i is the time away from 0
        //d is the change in angle over a set time
        
        pidController = new PIDController(gyro.getAngle(), 0, 0, gyro, frontRightMotor);
        
        
    }

    public void Airsystem() {
        airCompressor = new Compressor(1);  //Digtial I/O,Relay
        airCompressor.start();                        // Start the air compressor

        s1 = new Solenoid(1);                        // Solenoid port
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
    
    public void teleopPeriodic() {
        
        System.out.println(stick.getX() + ", " + stick.getY());
        double inputX = joystickZeroed(stick.getX());
        double inputY = joystickZeroed(stick.getY());
        double inputZ = joystickZeroed(stick.getZ());
        
        myRobot.mecanumDrive_Cartesian(-inputX, -inputY, -0.35*inputZ, 0);
        
//        double x = stick.getX();
//        
//        frontLeftMotor.set(x);
//        frontRightMotor.set(x);
//        rearLeftMotor.set(x);
//        rearRightMotor.set(x);

                
        Timer.delay(0.005);
    }
    
    public void testPeriodic() {        
        
        //test3();
    	//test4();
    	test2();
    	lift();
    	PID();
        
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
        frontRightMotor.set(speed);
        rearRightMotor.set(speed);
        rearLeftMotor.set(speed);
    }
    
    public void test3() {
    	
    	//s1.set(true);
    	double[] array = new double[4];
    	
    	wheel1Encoder.startLiveWindowMode();
    	//wheel2Encoder.startLiveWindowMode();
    	//wheel3Encoder.startLiveWindowMode();
    	//wheel4Encoder.startLiveWindowMode();
    	Timer.delay(1);
    	
    	array[0] = wheel1Encoder.getDistance();
    	lowest1 = wheel1Encoder.getDistance();
    	wheel1Encoder.getDirection();
//    	
//    	array[1] = wheel2Encoder.getDistance();
//    	wheel2Encoder.getDirection();
//    	
//    	array[2] = wheel3Encoder.getDistance();
//    	wheel3Encoder.getDirection();
//    	
//    	array[4] = wheel4Encoder.getDistance();
//    	wheel4Encoder.getDirection();
    	
    	wheel1Encoder.stopLiveWindowMode();
//    	wheel2Encoder.stopLiveWindowMode();
//    	wheel3Encoder.stopLiveWindowMode();
//    	wheel4Encoder.stopLiveWindowMode();
    	
//    	lowest = array[0];
//    	for(int i = 0; i < 4; i++){
//    		for(int j = 0; j < 4; j++){
//    			if(array[i] < array[j]){
//    				lowest = array[j];
//    			}
//    		}
//    		
//    	}
    	
//    	double ratio1 = array[0]/lowest;
//    	double ratio2 = array[1]/lowest;
//    	double ratio3 = array[2]/lowest;
//    	double ratio4 = array[3]/lowest;
    	
    	lowest1 = wheel1Encoder.getDistance();
    	System.out.println(lowest1-lowest2);
    	
    	lowest2 = lowest1;
    	
//    	frontLeftMotor.set(speed/ratio1);
//    	frontRightMotor.set(speed/ratio2);
//    	rearLeftMotor.set(speed/ratio3);
//    	rearRightMotor.set(speed/ratio4);
    	
    }
    
    public void test4() {
    	System.out.println(gyro.getAngle());
    	gyroPID();
    }
    
    
    /**
     * Auto corrects the driving of the robot to drive straight
     */
    public void gyroPID(){
    	double gyroAngle = gyro.getAngle();
    	
    	if(gyroAngle > 0){  //turn left if turned right
    		frontRightMotor.set(speed*2);
    		rearRightMotor.set(speed*2);
    	}
    	if(gyroAngle < 0){ //turn right if turned left
    		frontLeftMotor.set(speed*2);
    		rearLeftMotor.set(speed*2);
    	}
    	else{ //set all the wheels to the same speed if straight
    		frontRightMotor.set(speed);
    		rearRightMotor.set(speed);
    		frontLeftMotor.set(speed);
    		rearLeftMotor.set(speed);
    	}
    	
    	
    }
    
    public void lift() {
    	boolean applyBrake = true;
    	boolean liftUp = stick.getRawButton(1);
    	System.out.println(liftUp);
    	boolean liftDown = stick.getRawButton(2); //Not sure which buttons these are on the controller
    	System.out.println(liftDown);
    	
    	if(applyBrake == true) {
    		
    	}
    	
    	if(liftUp == true) {
    		applyBrake = false;
    		liftMotor.set(speed);
            Timer.delay(delay);
            liftMotor.set(STOP);
    	}
    	else {
    		applyBrake = true;
    	}
    	
    	if(liftDown == true) {
    		applyBrake = false;
    		liftMotor.set(reverse);
            Timer.delay(delay);
            liftMotor.set(STOP);
    	}
    	else {
    		applyBrake = true;
    	}
    }
    
    public void PID(){
    	
        //p is the gyro angle
        //i is the time away from 0
        //d is the change in angle over a set time
    	
    	if(gyro.getAngle() != 0){
    		if(isAngleZero == true){
    			PIDTime = 0;
    		}
        	// pidController = new PIDController(gyro.getAngle(), i, d, gyro, frontLeftMotor);
        	//pidController.setPID(gyro.getAngle(), i, d);
    	}

       
    }
    
}
