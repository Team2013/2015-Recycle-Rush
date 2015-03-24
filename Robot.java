package org.usfirst.frc.team2013.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CANTalon; //Imports for classes
import edu.wpi.first.wpilibj.Compressor; 
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;


public class Robot extends SampleRobot {
	private Compressor c = new Compressor(1);//Compressor
	//private Timer auto;
	
    
	
	
	private Timer auto;
	private Victor vicRight;//Initialization of Speed controllers, Solenoids and other items.
	private Victor vicLeft;
	private Talon toteMotor;
    private Talon armExtend;
    private Talon armMove;
    private Talon armWrist;
    private Solenoid daClaw;
    private Solenoid shifter;
    private Solenoid slicers;
    public Encoder lDrive;
    public Encoder rDrive;
    public Encoder armOut;
    private Potentiometer armPot;
    private AnalogInput ai;
    //CameraServer server;
    boolean burnout =false;
    double x=0;
    double y=0;
    double z=0;
    double armZ=0;
    double armX=0;
    double tm=1;
    double num;
    double ts=1;
    public int autoNumber=0;
    
    
    double current;
    int autoNum;
    boolean auto1;
  boolean  shift=false;
    boolean turnaround= false;
    boolean stop = false;
    boolean armLimit;
    boolean bind=false;
    
   // double[] motorValues;//0=left 1=right
    
   DigitalInput up = new DigitalInput(7);
   DigitalInput armIn = new DigitalInput(6);
    private Joystick stick; //Joystick
    private Joystick arm;
    
    //private Joystick buttonBoard;
    private PowerDistributionPanel pdp = new PowerDistributionPanel();
	private final double k_updatePeriod = 0.1; // update every 0.005 seconds/5 milliseconds (200Hz)
	boolean turn1=false;
	boolean back1 = false;
	//PIDController lside= new PIDController(0.01,1,3,lDrive,motorone);
	//PIDController rside=new PIDController(0.01,1,3,rDrive,motorthree);
    public Robot() {
        

//Creation of CAN Talons for motors.
        armLimit= armIn.get();
        toteMotor = new Talon(1);
        armMove= new Talon(3);
        armExtend= new Talon(0);
    	armWrist= new Talon(2);
    	vicLeft = new Victor(6);
    	vicRight = new Victor(8);
    	
    	
        
        auto= new Timer();
        shifter = new Solenoid(1);
        daClaw= new Solenoid(2);
        slicers = new Solenoid(3);
        lDrive = new Encoder(0,1, true, Encoder.EncodingType.k4X);
        rDrive = new Encoder(2,3,true, Encoder.EncodingType.k4X);
        armOut= new Encoder(4,5,true, Encoder.EncodingType.k2X);
        ai= new AnalogInput(1);
        
        armPot= new AnalogPotentiometer(ai,360,0);
        stick = new Joystick(0);// initialize the joystick on port 0    //buttonBoard= new Joystick(1);
        arm = new Joystick(1);
       // server = CameraServer.getInstance();
      //  server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        //server.startAutomaticCapture("cam0");
        c.setClosedLoopControl(true);//Compressor Set-Up
        lDrive.setDistancePerPulse(0.00028264);//This is high gear, low is ldrive.setDistancePerPulse(0.00064254);
        rDrive.setDistancePerPulse(0.00028264); //60:24 gear with 40:18
        rDrive.setSamplesToAverage(127);
        lDrive.setSamplesToAverage(127);
      
        
        
    }public void drive(double vY,double vX,double wZ){
    	double leftM= (wZ+vY+vX)*0.45;
    	double rightM= (wZ-vY+vX)*0.45;
    	vicRight.set(rightM);
    	vicLeft.set(leftM);
    }
    
    
    
    public void ramp(double a,double b){
    	double ramp=a;
    	if(ramp>b+0.15){
    		ramp-=0.15;
    	}else if(ramp<b-0.15){
    		ramp+=0.15;
    	}else{
    		ramp=b;
    	}
    	armMove.set(ramp);
    }
   
    
    public void autonomous(){
    	 lDrive.reset();
    	 rDrive.reset();
    
    	 auto.reset();
   while(isAutonomous()&&isEnabled()){
	   auto.start();
	   int autoChoice=0;
	   boolean godMode=false;
	   double wrist=0.5;
	   int addUp=0;
	   boolean finished1=false;
	   boolean finished2=false;
	   boolean finished3=false;
	   Timer boot= auto;
	   armWrist.set(0.15);
	   if(godMode==true){
		   if(lDrive.getDistance()<3&&finished1==false){
			   drive(1,0,0);
			   finished1=true;
		   }else if(lDrive.getDistance()>3&&finished2==false){
			  drive(0,0,1);
			  finished2=true;
		   }else if(lDrive.getDistance()>3.5&&finished3==false){
			   boot.start();
			   drive(0,0,0);
			   
			   for(int i=0;i<5;i++){
				   boot.delay(0.5);
				   armWrist.set(wrist*-1);
				   addUp++;
			   }
			   
		   }else if(addUp==5){
			   drive(0,0,-1);
			   finished3=true;
			   armWrist.set(0);
			   
		   }else if(auto.equals(8)){
			   drive(0,0,0);
			   addUp=0;
			   for (int i=0;i<5;i++){
				   auto.delay(0.5);
				   armWrist.set(wrist*-1);
				   
			   }
		   }
	   }else if(autoChoice==1){//This is the step can grab auto
		   if(lDrive.getDistance()*-1<1.1){
			   vicLeft.set(1);
			   vicRight.set(1);
		   }else if(lDrive.getDistance()*-1>1){
			   vicLeft.set(-1);
			   vicRight.set(1);
		   }else if(lDrive.getDistance()*-1>2){
			   drive(1,0,0);
	   }
	   
   }else if(autoChoice==3){
	   int i=0;
	   int b=0;
	   if(i<50){
	   for( i=0;i<50;i++){
		   drive(0,0,0.5);
		   toteMotor.set(0.2);
		   armWrist.set(0.2);
	   }}
	   if(i==50){
		   for(b=0;b>50;b++){
			   drive(0.5,0,0);
			   toteMotor.set(0);
			   
		   }
	   }else if(b==50){
		   drive(0,0,0);
		   armWrist.set(0);
		   
	   }
   }
 
    }
    }

    
    public void operatorControl() {
    	
    	rDrive.reset();
    	lDrive.reset();
    	armOut.reset();
    	
    	daClaw.set(false);
    	
    	
        while (isOperatorControl() && isEnabled()) {
        	double armpot=0;
        	boolean lock =false;
        	if(stick.getRawButton(1)){
            	shifter.set(false);
            
        	}else{
            	shifter.set(true);
            	
        	}
        		current = pdp.getCurrent(4);
        		armLimit=armIn.get();
        		y=stick.getY();
        		x=0;
        		z=stick.getZ()*0.5;
        		if(stick.getRawButton(2)){
        		drive(y,x,z);
        		}else{
        			drive(0,0,0);
        		}
        		armX=arm.getRawAxis(3);
        		armZ=arm.getZ()*0.625;
        		armWrist.set(armZ);
        		ramp(armMove.get(),armX);
        		
        		
        		
        
        		if(arm.getRawButton(7)){
        			daClaw.set(true);
        		}else if(arm.getRawButton(8)){
            		daClaw.set(false);
            	}else{}
        		
        		if(arm.getRawButton(1)){
            		slicers.set(true);
            	}else if(arm.getRawButton(2)){
            		slicers.set(false);
            	}else{}
        		
        		
            	if(stick.getPOV()==0){
            		toteMotor.set(1);
            		
            	}else if(stick.getPOV()==180){//tote stacker down control
            		toteMotor.set(-1);
            	}else{
            		tm=0.05;
            		toteMotor.set(tm);
            		burnout=false;
            	}
            	if(arm.getRawButton(5)){
            		
            		armExtend.set(-1);	//arm extension out
            		stop=false;
            	}else if(arm.getRawButton(6)){
            	
            		armExtend.set(1);
            		//arm extension in
            	}else{
            		
            		armExtend.set(0);
            		
            	}if(arm.getRawButton(12)){
            		armpot=armPot.get();
            		lock=!lock;
            	}
            	if(armOut.get()>427){
        			stop=true;
        			
        		}
        		
        	if(stick.getRawButton(4)){//this is the 180 degree drive for stacking bins onto totes
        	lDrive.reset();
        	rDrive.reset();
        	turnaround=true;
        	}if(turnaround==true){
        		shifter.set(true);
        		if(rDrive.getDistance()<1){
        		drive(-1,0,0);//drive backwards
        		}
        		
        			else if(rDrive.getDistance()>1&&rDrive.getDistance()<3){
        			drive(-1,0,-1);//drive in a circle
        		}
        				else if(rDrive.getDistance()>3&&rDrive.getDistance()<4){
        			drive(1,0,0);//drive forward
        		}
        					else {
        			drive(0,0,0);//stop
        			turnaround=false;
        			shifter.set(false);
        		}
        		
        		}if(armOut.get()>427){
        			stop=true;
        			
        		}if(current>30){
            		burnout=true;//motor amperage control
            	} else{
            	
            		if(arm.getZ()==0){
                		armWrist.set(0);}
            	}if(lock==true){
            		if(armPot.get()<armpot){
            			armMove.set(armpot-armPot.get());
            		}else if(armPot.get()>armpot){
            			armMove.set(armpot-armPot.get());
            		}else{
            			armMove.set(0);
            		}
            	}else if(lock==false){
            		armpot=0;
            	}
        	
        	
            Timer.delay(k_updatePeriod);	// wait 5ms to the next update
            //double current = toteMotor.getOutputCurrent();//Inputs from pdp and pcm
            
            //
            
            double voltage = pdp.getVoltage();
            double temp = pdp.getTemperature();
            double power = pdp.getTotalPower();
            float compressor = c.getCompressorCurrent();
            double leftDrive = lDrive.getDistance();
            double rightDrive = rDrive.getDistance();
            double pot= ai.getVoltage();
            boolean upLimit = up.get();
            double armout = armOut.get();
             autoNum=0;
           // boolean downLimit= down.get();
            SmartDashboard.putNumber("Channel 13 Current: ", current);//Out puts to the smart Dashboard. 
            
            SmartDashboard.putNumber("Total Power Output: ", power);
            SmartDashboard.putNumber("Compressor Current", compressor);
            SmartDashboard.putNumber("Left Encoder", leftDrive);
            SmartDashboard.putNumber("Right Encoder", rightDrive);
            SmartDashboard.putBoolean("Down Limit", upLimit);
            SmartDashboard.putNumber("Arm encoder",armout );
            SmartDashboard.putBoolean("Arm Limit",armLimit);
            SmartDashboard.putNumber("ArmPot",pot);
            SmartDashboard.putNumber("3rd Axis",stick.getZ());
            SmartDashboard.putBoolean("Lock ",lock);
            SmartDashboard.putNumber("2nd Axis",stick.getX());
            SmartDashboard.putNumber("1st Axis",stick.getY());
            SmartDashboard.putNumber("Channel 12", current);
           // SmartDashboard.putBoolean("Down Limit", downLimit);
          //  SmartDashboard.putBoolean("trigger boolean", turnaround);
            
            
            
        
        	
        }
    }
}
