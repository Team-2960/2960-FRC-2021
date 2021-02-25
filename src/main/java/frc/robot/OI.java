package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.SubSystems.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Camera.Camera;
public class OI extends SubsystemBase{
    private Climb climb;
    private Intake intake;
    private Drive drive;
    private Shooter shooter;
    public Pivot pivot;
    private Index index;
    private Camera camera;
    private MEGAShooter mShooter;
    //Joysticks
    private Joystick driver_Control;
    private Joystick operator_Control;
    private Joystick backUp_Control;
    public OI(){
        //Init Classes
        camera = Camera.get_Instance();
        drive = Drive.get_Instance();
        climb = Climb.get_Instance();
        intake = Intake.get_Instance();
        shooter = Shooter.get_Instance();
        pivot = Pivot.get_Instance();
        index = Index.get_Instance();
        mShooter = MEGAShooter.get_Instance();
        //joysticks
        driver_Control = new Joystick(Constants.driver_Control_Left);
        
        operator_Control = new Joystick(Constants.operator_Control);
        backUp_Control = new Joystick(Constants.backUp_Control);
    }
    /**
     * Commands for the joysticks
     * @param driver_Control joystick 1
     * @param joystick2 joystick 2
     */
    public void Controller(){ 
        //manual intake control
        if(backUp_Control.getRawButton(11)){
            intake.setPosition(0);
        }else if(backUp_Control.getRawButton(12)){
            intake.setPosition(1);
        }

        /*****************************/
        /*    manual pivot control   */
        /*****************************/
        
        if(isManualControl()){ //this may change
            pivot.DisablePivotPID();
            pivot.SetPivotSpeed(backUp_Control.getRawAxis(1));
        }else{
            if(dTrenchHeight() || oIntakeOut()){// driver intake posistion and operator posistion
                //sets intake down ready to intake
                mShooter.intakePosition();  // this will move the intake down and pivot down
            }else if(oShortPreset()){ // short preset
                mShooter.ShortShoot(); // go to target angle and rate
            }else if(oLongPreset()){ //long preset
                mShooter.longShoot(); // go to target angle and rate   
            }else if(driver_Control.getRawButton(1)){
                pivot.setPTargetAngle(145);
                shooter.gotoRate(-7250);
                if(dShootOut() && driver_Control.getRawButton(2)){
                    mShooter.shootAlways(-7250);
                }

            }
                /*else if(oCameraTracking()){
                mShooter.wheelOfFortunePreset(); //go to target angle for wof
            } */else{ 
                mShooter.toNeuturalPosition(); // go to netural position
                shooter.setShooterSpeed(0, 0); // stop shooter
            }

            if(oIntakeIn()) {  
                mShooter.intakeUp();  //move the intake up
            }
        }
        //Driver Control
        //TODO: May be need a deadband
        if(dAlignDrive() || oCameraTracking()){
            drive.targetLineUp();
            drive.anglePID = true;
        }
        else if(driver_Control.getRawButton(6)){
            drive.anglePID = false;
        }
        else{
           //drive.setArcDriveRate(50, driver_Control.getRawAxis(5));
            drive.setSpeed(driver_Control.getRawAxis(1)* -0.75, driver_Control.getRawAxis(5)* -0.75); //drive
        }
        if(driver_Control.getRawButton(6)){
            drive.resetAngle();
        }
        if(dintakeIn()){ //start intake
            mShooter.intakeEnableDr();
        }else if(oIndexOut()){ //index out take
            index.setSpeed(-1, -1);
        }else if(dintakeOut()){ //intake and shooter out take
            intake.setSpeed(-1); //out
            shooter.setShooterSpeed(-0.4, -0.4); 
        }else if(oShortPreset() && dShootOut()){ //auto shoot ball
            mShooter.dShortShoot();
        }else if(oLongPreset() && (dShootOut())){ // auto shoot ball
            mShooter.dLongShoot();
        }else if(backUp_Control.getRawButton(4)){
            shooter.gotoRate(Constants.shortPreset[0]);
        }/* else if(oCameraTracking()&& dShootOut()){
            shooter.setShooterSpeed(0.1, -0.1); //run the shooter wof
        } */
        
        
        
        else if(backUp_Control.getRawButton(3)){
            shooter.gotoRate(-6000);
        }
        
        
        
        else{//stop intake and index
            intake.setSpeed(0);
            index.disableIndex();
        }

        //winch
        if(oWinching()) { //winch in
            climb.setSpeed(-0.9);
        }else if(dBackWinching()) { // winch out
            climb.setSpeed(0.2);
        }else{
            climb.setSpeed(0); //off
        }
    
        if(oClimbExtended()){
            climb.setPosition(0);
        }
        else if(oClimbRetracted()){
            climb.setPosition(1);
        }

    }
    
    /**
     * put to smartdashboard
     */
    public void SmartDashboard(){
        //UNCOMMENT FUNCTION
        //SmartDashboard.putNumber("calc angle", (camera.calcAngle(camera.getCenterX()) + drive.getAngle()));
        //SmartDashboard.putNumber("togo angle", (camera.calcAngle(camera.getCenterX())));
        //SmartDashboard.putNumber("", (camera.calcAngle(camera.getCenterX())));
        SmartDashboard.putNumber("battery volt", RobotController.getBatteryVoltage());

    }
    /**
     * Run every time
     */
    public void periodic(){
        if(DriverStation.getInstance().isOperatorControl()){
            Controller();
        }
    }

    











    /** 
     * @return boolean
     */
    // Driver Control Outline
    private boolean dintakeIn(){
        return driver_Control.getRawAxis(3) > 0.1;
    }
    
    /** 
     * @return boolean
     */
    private boolean dintakeOut(){
        return driver_Control.getRawAxis(2) > 0.1;
    }

    private boolean intakeDisable(){
        return driver_Control.getRawButton(17);
    }
    
    /** 
     * @return boolean
     */
    private boolean dTrenchHeight(){
        return driver_Control.getRawButton(5) && driver_Control.getRawButton(6);
    }
    
    /** 
     * @return boolean
     */
    private boolean dShootOut(){
        return driver_Control.getRawButton(1) || operator_Control.getRawButton(11);
    }
    
    /** 
     * @return boolean
     */
    private boolean dAlignDrive(){
        return driver_Control.getRawButton(5);
    }
    
    /** 
     * @return boolean
     */
    private boolean dPivotAlign(){
        return driver_Control.getRawButton(6);
    }

    
    /** 
     * @return boolean
     */
    // Operator Control Outline
    //is the robot in manual control
    private boolean isManualControl(){
        return operator_Control.getRawButton(1);
    }
    
    /** 
     * @return boolean
     */
    //is climb extended
    private boolean oClimbExtended(){
        return operator_Control.getRawButton(3);
    }
    
    /** 
     * @return boolean
     */
    //is climb retracted
    private boolean oClimbRetracted(){
        return operator_Control.getRawButton(2);
    }
    
    /** 
     * @return boolean
     */
    //is winch winching
    private boolean oWinching(){
        return operator_Control.getRawButton(4);
    }
    private boolean dBackWinching(){
        return driver_Control.getRawButton(7) && driver_Control.getRawButton(8);
    }
    
    /** 
     * @return boolean
     */
    //is the robot facing front
    private boolean isPivotFront(){
        return operator_Control.getRawButton(5);
    }
    
    /** 
     * @return boolean
     */
    //is the pivot Camera tracking
    private boolean oCameraTracking(){
        return operator_Control.getRawButton(6);
    }
    
    /** 
     * @return boolean
     */
    //is the short preset pressed
    private boolean oShortPreset(){
        return operator_Control.getRawButton(7);
    }
    
    /** 
     * @return boolean
     */
    //is the long pressed
    private boolean oLongPreset(){
        return operator_Control.getRawButton(8);
    }
    
    /** 
     * @return boolean
     */
    //is the Intake in
    private boolean oIntakeOut(){
        return operator_Control.getRawButton(9);
    }
    
    /** 
     * @return boolean
     */
    //is the Intake out
    private boolean oIntakeIn(){
        return operator_Control.getRawButton(10);
    }
    private boolean oFeederStation(){
        return (!oIntakeIn() && (!oIndexOut()));
    }
    
    /** 
     * @return boolean
     */
    //Is the shoot pressed
    private boolean oShoot(){
        return operator_Control.getRawButton(11);
    }
    
    /** 
     * @return boolean
     */
    //is the indexer indexing out
    private boolean oIndexOut(){
        return operator_Control.getRawButton(12);
    }
}


















/* 
Driver:
    drive
    intake: in / out (balls)
    shoot



Operator:
    intake out:
        -intake down
        -pivot too there(need wait)

    intake in:
        -




















*/