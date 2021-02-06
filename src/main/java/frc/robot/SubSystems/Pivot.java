package frc.robot.SubSystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.Trapezoid;
import frc.robot.Camera.*;

public class Pivot extends SubsystemBase{
    public static Pivot pivot;
    //private Camera camera;
    //Pivot motor
    private CANSparkMax mLeftPivot;
    private CANSparkMax mRightPivot;

    //Pivot PID controler
    private PIDController aPidController;
    private ArmFeedforward armfeedforward;

    private boolean isPivotEnabled = false;
    private boolean cameraTrackingEnabled =false;
    private Trapezoid trapezoid;  
    private double pivotTarget;
    private boolean isPivotFront;
    public int lookUpPos;
    public int pivotTablePos = 0;
    //encoder
    private Encoder pEncoder;
    private DutyCycleEncoder pabsEncoder;
    private double angleOffset;
    /** 
     * @return Pivot
     */
    public static Pivot get_Instance(){
        if(pivot == null){
            pivot = new Pivot();
        } 
        return pivot;
    }

    private Pivot(){
        SmartDashboard.putNumber("Vout", 0);


        //init code
        mLeftPivot = new CANSparkMax(Constants.mLeftPivot, MotorType.kBrushless);
        mRightPivot = new CANSparkMax(Constants.mRightPivot, MotorType.kBrushless);
        //init camera
        //camera = Camera.get_Instance();
  
        //encoder 
        pEncoder = new Encoder(Constants.pEncoder1, Constants.pEncoder2, false, CounterBase.EncodingType.k4X);
        pEncoder.setMaxPeriod(.1);
        pEncoder.reset();

        pabsEncoder = new DutyCycleEncoder(0);
        pabsEncoder.setDistancePerRotation(360);
        //Arm PID Setup
        aPidController = new PIDController(Constants.pKp, Constants.pKi, Constants.pKd);
        armfeedforward = new ArmFeedforward(Constants.pKs, Constants.pKcos, Constants.pKv, Constants.pKa);
        mLeftPivot.setInverted(false);
        mRightPivot.setInverted(true);//THIS IS FALSE IF THEY ARE FIGHTING REVERT THIS ONE

        //System.out.println("Pivot Class created");
    }
    
    /** 
     * @param front
     */
    public void setpivotDirection(boolean front){
      isPivotFront = front;
      if(front){
        lookUpPos = 1;
      }
      else{
        lookUpPos = 2;
      }
    }
    /**
     * Set the motor speed
     * @param speed set speed
     */
    public void SetPivotSpeed(double speed){
      mLeftPivot.set(speed);
      mRightPivot.set(speed);
    }
    /**
     * set the motor rate with pid
     * @param rate target rate
     */
    public void SetPivotPIDRate(double rate){
        double pid_output = aPidController.calculate(pEncoder.getRate(), rate);
        double feedforward = armfeedforward.calculate(pabsEncoder.getDistance(), 0) / RobotController.getBatteryVoltage();
        double speed = pid_output  + feedforward ;
        SetPivotSpeed(speed);
    }

    /**
     * set the target
     * @param target set target
     */
    public void setPTargetAngle(double target){
      EnablePivotPID();
      if(pivotTarget != target){
        pivotTarget = target;
        trapezoid = new Trapezoid(150, -150, 2, 300, -300, -800, 800, pabsEncoder.getDistance(), pivotTarget, pEncoder.getRate(), 28, 28);
      }
    }
  
    /**
     * go to target angle
     * @param angle
     */
    private void gotoAngle(){
      double rate = trapezoid.trapezoidCalc(pabsEncoder.getDistance(), mLeftPivot.getEncoder().getVelocity());
      SetPivotPIDRate(rate);
    }
    
      
    
    /** 
     * @return boolean
     */
    public boolean atPivotTarget(){
      double error = Math.abs(pabsEncoder.getDistance() - pivotTarget);
      return error < Constants.angleTolerance;
    }
    
    /** 
     * @return boolean
     */
    public boolean pivotInWindow(){
      boolean isInWindow = false;
      if(isPivotFront){
        if(pabsEncoder.getDistance() > Constants.frontWindowMin && pabsEncoder.getDistance() < Constants.frontWindowMax){
          isInWindow = true;
        }
      }
      else{
        if(pabsEncoder.getDistance() > Constants.backWindowMin && pabsEncoder.getDistance() < Constants.backWindowMax){
          isInWindow = true;
        }
      }
      return isInWindow;
    }
    public double getPivotPos(){
      return pabsEncoder.getDistance();
    }
    
    /** 
     * @return double
     */
    public double frontOrBack(){
      double neuturalPos;
      if(isPivotFront){
        neuturalPos = Constants.neuturalPosFront;
      }
      else{
        neuturalPos = Constants.neuturalPosBack;
      }
      return neuturalPos;
    }
    
    /** 
     * @param pos
     */
    //UNCOMMENT FOR CAMERA FUNCTION
/*     public void pivotToTarget(int pos){
      if(cameraTrackingEnabled){
        if(!pivotInWindow() || !camera.isTargetFound()){
            setPTargetAngle(frontOrBack());
        }    
        
        else{
          setPTargetAngle(Constants.pivotTable[pos][lookUpPos]);
        }
      }
    } */
    
    /** 
     * @param offset
     */
    /* public void pivotAngleOffset(double offset){
      angleOffset= 100 * offset;
      setPTargetAngle(pivotTarget + angleOffset);
    } */
    public void smartdashboard(){
      SmartDashboard.putNumber("Encoder Value Rate", pEncoder.getRate());
      SmartDashboard.putNumber("target position pivot", pivotTarget);
      SmartDashboard.putNumber("ABS Encoder Value Degrees", pabsEncoder.getDistance());
      SmartDashboard.putNumber("Left motor Speed: ", mLeftPivot.get());
      SmartDashboard.putNumber("Right motor Speed: ", mRightPivot.get());
      SmartDashboard.putNumber("Left motor Temp: ", mLeftPivot.getMotorTemperature());
      SmartDashboard.putNumber("Right motor Temp: ", mRightPivot.getMotorTemperature());
      SmartDashboard.putNumber("Left motor Current: ", mLeftPivot.getOutputCurrent());
      SmartDashboard.putNumber("RIght motor Current: ", mRightPivot.getOutputCurrent());
      //SmartDashboard.putNumber("",)
    }
    
    /**
     * run every time
     */
    public void periodic() {
      //Timer time = new Timer();
      //time.start();
      // This method will be called once per scheduler run
      //smartdashboard();
      //enable pivot PID
      //UNCOMMENT FOR CAMERA
     /*  double distance = camera.getTargetDistance();
     SmartDashboard.putNumber("Distance", distance);
     if(cameraTrackingEnabled){
       pivotTablePos = 0;
       while(distance > Constants.pivotTable[pivotTablePos][0] && Constants.pivotTable[pivotTablePos][0] < Constants.pivotTable.length){
         pivotTablePos++; 
        }
        double under = distance - Constants.pivotTable[pivotTablePos][0];
        double above = 0;
        try{
          above = distance - Constants.pivotTable[pivotTablePos + 1][0];
        }
        catch(Exception e){
          above = under;
        }
        
        if(above< under){
          pivotTablePos = pivotTablePos + 1;
        }
        pivotToTarget(pivotTablePos);
      }
      
      
      */
      
      if(isPivotEnabled){
        gotoAngle();
      }
      //SmartDashboard.putNumber("Pivot Timer",  time.get());
    }
  /**
   * enable the pivot pid
   */
  public void EnablePivotPID(){
    isPivotEnabled = true;
  }
  /**
   * disable the pivot pid
   */
  public void DisablePivotPID(){
    isPivotEnabled = false;
  }

  
  /** 
   * @param isEnabled
   */
  public void isCameraTrackingEnabled(boolean isEnabled){
    cameraTrackingEnabled = isEnabled;
  }
  public double pivotTarget(){
    return pivotTarget;
  }
}