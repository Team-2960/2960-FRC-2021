package frc.robot.SubSystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Index extends SubsystemBase {
  private static Index index;
  public static CANDigitalInput photoeye;
  
  private static CANSparkMax mRightIndex;
  private static CANSparkMax mLeftIndex;

  private int isIndexEnabled = 0;
  private boolean isAutoIndexEnabled = false;

  /**
   * @return Index
   */
  public static Index get_Instance() {
    if (index == null) {
      index = new Index();
    }
    return index;
  }

  private Index() {
    mLeftIndex = new CANSparkMax(Constants.mIndex1, MotorType.kBrushless);
    mRightIndex = new CANSparkMax(Constants.mIndex2, MotorType.kBrushless);
    photoeye = new CANDigitalInput(mLeftIndex, CANDigitalInput.LimitSwitch.kForward,
        CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
    mRightIndex.getEncoder().setPosition(0);
  }

  /**
   * Gets the photo eye and prints it
   */
  public void setSpeed(double right, double left) {
    mLeftIndex.set(left);
    mRightIndex.set(-right);
  }
  private void startIndexIn(){
      if(photoeye.get()){
        setSpeed(0.9, 0.75);// 1, 0.85
      }
      else{
        setSpeed(0, 0);
      }
  }
  private void startIndexOut(){
    if(!photoeye.get()){
      setSpeed(-1, -1);
    }
    else{
      if(photoeye.get()){
        setSpeed(-1, -1);
      }
      else{
        setSpeed(0, 0);
      }
    }
  
  
  }
  public boolean indexBeltsGoneDistance(double distance){
    return distance < getEncoderDistance();
  }
  public static boolean getPhotoeyeIndex(){
    return photoeye.get();
  }
  public void SmartDashboard(){
    SmartDashboard.putBoolean("indexPhoto", photoeye.get());
    SmartDashboard.putNumber("Right Motor i", mRightIndex.get());
    SmartDashboard.putNumber("left Motor i", mLeftIndex.get());
  }
  @Override
  public void periodic() {
    //Timer time = new Timer();
    //time.start();
    // This method will be called once per scheduler run
    // This method will be called once per scheduler run 
    SmartDashboard();
    /*if(isAutoIndexEnabled){
      if(isIndexEnabled == 1){
        startIndexIn();
      }else if(isIndexEnabled == -1){
        startIndexOut();
      }
    }*/
    //SmartDashboard.putNumber("IndexTimer",  time.get());
  }

  
  /** 
   * @param dirction
   */
  public void enableIndex(int dirction){
    isAutoIndexEnabled = true;
    isIndexEnabled = dirction;
  }
  
  public void disableIndex(){
    isAutoIndexEnabled = false;
    setSpeed(0, 0);
    isIndexEnabled = 0;
  }
  public double getEncoderDistance(){
    return mRightIndex.getEncoder().getPosition();
  }
}
