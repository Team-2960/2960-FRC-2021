package frc.robot.SubSystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubSystems.*;
import frc.robot.Constants;
import frc.robot.Camera.Camera;



public class MEGAShooter extends SubsystemBase {
  private static MEGAShooter megaShooter;
  private Intake intake;
  private Shooter shooter;
  private Pivot pivot;
  private Index index;
  private Camera camera;
  private boolean shoot = false;
  public double speed = -6000;
  public boolean setMotorToIntake = false;
  public Timer shootTimer;
  public Timer afterShoot;
  /** 
   * @return megaShooter
   */
  public static MEGAShooter get_Instance(){
    if(megaShooter == null){
      megaShooter = new MEGAShooter();
    } 
    return megaShooter;
  }

/**
 * MEGAShooter costructor
 */
  private MEGAShooter() {
    //camera = Camera.get_Instance();
    intake = Intake.get_Instance();
    shooter = Shooter.get_Instance();
    pivot = Pivot.get_Instance();
    index = Index.get_Instance();
    shootTimer = new Timer();
    afterShoot = new Timer();
  }
  
  /** 
   * allows for the offset for shooter and the pivot to be used
   * @param angle the value from the joystick that gets passed
   * @param speed the value from the joystick that gets passed 
   */
  public void setOffset(double angle, double speed){
    speed = (speed - 0.50) * 2;// makes the value between -1 and 1
    angle = (angle - 0.50) * 2;// makes the value between -1 and 1
    shooter.setSpeedOffset(speed);//sets the shooter offset 
/*     pivot.pivotAngleOffset(angle);//sets the pivto offset
 */  }




 
 /**
  * disables the manual control
  */
  public void disableManual(){
    shooter.setSpeedOffset(0);//sets the shooter speed to zero
    /* pivot.pivotAngleOffset(0); *///resets the pivot offset
  }



  /**
   * 
   */
  public void intakeEnableDr(){
    //intake in
    index.startIndexIn();
    intake.setSpeed(1);
    shooter.gotoRate(4000);
  }


  //new function to review
  public boolean pivotToBumper(){
    if(pivot.getPivotPos() < 280 - Constants.angleTolerance){
      pivot.setPTargetAngle(280);     // TODO: Move PID stop position to constants
      return false;
    }else if(pivot.getPivotPos() < 319 - Constants.angleTolerance){ // TODO: Move hardstop position to constants
      pivot.DisablePivotPID();
      pivot.SetPivotSpeed(0.2);
      return false;
    }else{
      pivot.DisablePivotPID();
      pivot.SetPivotSpeed(0);
      return true;
    }
  }


  public void intakeOutEnableDr(){
    intake.setSpeed(-1);
  }
   /**
   * uses the to bumper to tell the pivot when to move and when to not
   */
  public void intakePosition(){
    if(intake.isIntakeOut()){//uses the intake out function to tell when it is out
      if(intake.getTime() > 0.5){   // TODO: Move Intake Delay to constants        
        if(pivotToBumper()){ 
          pivot.DisablePivotPID();// if the pivot is down at the bumper then it stops the pvot PID
        }
      }
    }else{
      intake.setPosition(1);//if the pivot is not at the bumper then it is putting the intake down
    }
  }
  
  
  /**
   * if the intake is up then it will set it to neutural
   */
  public void intakeUp(){
    if(pivot.pivotTarget() > 260){//if the pivot is in the way then the pivot will go to the neutural position
      toNeuturalPosition();
    }
    if(pivot.getPivotPos() < Constants.pivotOutOfReach){//if the pivot is out of the way then the intake will coem up
      intake.setPosition(0);
    }
  }




  //not correct
  /**
   * uses the ready to shoot function to tell when the shooter can shoot with acurracy
   * @param rate th rate
   */
  boolean bool = false;
  public void shootAlways(double rate){
    
    shooter.gotoRate(rate);
    if(!shoot){
    if(shooter.readyToShoot()){
      shoot = true;
      
    }
    else{
      shoot = false;
    }
    }if(!bool){
    if(shoot){
      bool = true;
      shootTimer.start();
      index.setSpeed(-1, -1);
    }
  }

    if(shootTimer.get() > 0.07){
      
      shootTimer.reset();
      shootTimer.stop();
      afterShoot.start();
      index.setSpeed(0, 0);
    }
    if(afterShoot.get() >0.2){
      shoot = false;
      bool=false;
      afterShoot.stop();
      afterShoot.reset();
    }



  }




  /**
   * sets the target position and the target rate that the pivot should be going to when shooting from the short shoot position
   */
  public void ShortShoot(){
    pivot.setPTargetAngle(Constants.shortPreset[1]);
    shooter.gotoRate(Constants.shortPreset[0]);
  }



  /**
   * sets the target position and the target rate that the pivot should be going to when shooting from the long shoot position
   */
  public void longShoot(){
    pivot.setPTargetAngle(Constants.longPreset[1]);
    shooter.gotoRate(Constants.longPreset[0]);
  }
  public void preset2(){
    pivot.setPTargetAngle(Constants.preset2[1]);
    shooter.gotoRate(Constants.preset2[0]);
  }
  public void preset3(){
    pivot.setPTargetAngle(Constants.preset3[1]);
    shooter.gotoRate(Constants.preset3[0]);
  }
  public void preset4(){
    pivot.setPTargetAngle(Constants.preset4[1]);
    shooter.gotoRate(Constants.preset4[0]);
  }



  /**
   * sets the target position to the neurural position
   */
  public void toNeuturalPosition(){
    pivot.setPTargetAngle(Constants.neuturalPosFront);
  }



  /**
   * shoots the balls at the short shoot
   */
  public void dShortShoot(){
    shootAlways(Constants.shortPreset[0]);
  }




  /**
   * shoots the balls at the long shoot
   */
  public void dLongShoot(){
    shootAlways(Constants.longPreset[0]);
  }
  public void dPreset2(){
    shootAlways(Constants.preset2[0]);
  }
  public void dPreset3(){
    shootAlways(Constants.preset3[0]);
  }
  public void dPreset4(){
    shootAlways(Constants.preset4[0]);
  }
  @Override
  public void periodic() {
    System.out.println(shoot + "shoot");
    //Timer time = new Timer();
    //time.start();
    // This method will be called once per scheduler run
   // SmartDashBoard();
    //SmartDashboard.putNumber("megaTimer",  time.get());
  }
}