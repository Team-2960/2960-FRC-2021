package frc.robot.SubSystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//UMCOMMENT CODE
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Camera.Camera;



public class Drive extends SubsystemBase {
  private static Drive drive;
  //motors
  private CANSparkMax mLeftMaster;
  private CANSparkMax mLeftFollow1;
  private CANSparkMax mLeftFollow2;
  
  private CANSparkMax mRightMaster;
  private CANSparkMax mRightMfollow1;
  private CANSparkMax mRightMfollow2;

  //PID Controller
  private PIDController drivePidController;

  private AHRS navX;
  //Encoders
  private CANEncoder rightEncoder;
  private CANEncoder leftEncoder;

  public double TargetDistance;
  public double TargetAngle;

  public double forwardSpeed;
  public double targetSpeed = 0;
  public int PIDCheck = 0;
  public boolean isDrivePIDEnabled = false;
  private static Camera camera;
  public double cameraAngle = 0;
  private double targetAngle = 0;
  public boolean anglePID;
  private double cameraCalcAngle = 0;
  
  
  /** 
   * @return Drive
   */
  public static Drive get_Instance(){
    
    if(drive == null){
      drive = new Drive();
    } 
    return drive;
  }

  private Drive() {
    //init code
    camera = Camera.get_Instance();
    //init all the motors
    mLeftMaster = new CANSparkMax(Constants.mLeftMaster1, MotorType.kBrushless);
    mLeftFollow1 = new CANSparkMax(Constants.mLeftFollow2, MotorType.kBrushless);
    mLeftFollow2 = new CANSparkMax(Constants.mLeftFollow3, MotorType.kBrushless);

    mRightMaster = new CANSparkMax(Constants.mRightMaster1, MotorType.kBrushless);
    mRightMfollow1 = new CANSparkMax(Constants.mRightMfollow2, MotorType.kBrushless);
    mRightMfollow2 = new CANSparkMax(Constants.mRightMfollow3, MotorType.kBrushless);

    //set master moter and follower motor
    mLeftFollow1.follow(mLeftMaster);
    mLeftFollow2.follow(mLeftMaster);
    mRightMfollow1.follow(mRightMaster);
    mRightMfollow2.follow(mRightMaster);

    //UNCOMMENT CODE
    navX = new AHRS(SPI.Port.kMXP);

    //init encoder
    rightEncoder = mRightMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, Constants.pulsePerRev);
    rightEncoder.setPositionConversionFactor(Constants.wheelcircumference);
    rightEncoder.setPosition(0);
    leftEncoder = mRightMaster.getAlternateEncoder(AlternateEncoderType.kQuadrature, Constants.pulsePerRev);
    leftEncoder.setPositionConversionFactor(Constants.wheelcircumference);
    leftEncoder.setPosition(0);

    //init drive PID
    drivePidController = new PIDController(Constants.dKp, Constants.dKi, Constants.dKd);
  }


  /**
   * Checks to see if we are at the target distance
   * @return true if we are at the target distance
   */
  public boolean checkDistance(){
    boolean isAtDistance;
    double error = Math.abs(encoderPosition() - TargetDistance);
    if(error < Constants.distanceTolerance){
      isAtDistance = true;
    }
    else{
      isAtDistance = false;
    }
    return isAtDistance;
  }
  /**
   * Checks to see if we are at the target distance
   * @return true if we are at the target angle
   */
  //UNCOMMENT FUNCTION

/*   public boolean checkAngle(){
    boolean isAtAngle;
    double error = Math.abs(getAngle() - TargetAngle);
    if(error < Constants.angleTolerance){
      isAtAngle = true;
    }
    else{
      isAtAngle = false;
    }
    return isAtAngle;
  }
  */
  public void targetLineUp(){
    cameraCalcAngle =camera.calcAngle(camera.getCenterX());
    setTargetAngle(navX.getAngle() - 2 * camera.calcAngle(camera.getCenterX()));
  }
  public void setTargetAngle(double angle){
    targetAngle = angle;
  }
  /**
   * Use the PID to set the rate we are going
   * @param rate is the target angle rate we want to be going
   */
  //UNCOMMENT FUNCTION
/*   public void setDriveRate(double rate){
    double speed = drivePidController.calculate(navX.getRawGyroX(), rate); //calc the speed
    SmartDashboard.putNumber("speed", speed);
    setSpeed(-speed, speed);
  } */
  /**
   * gives the PID the numbers that we want to be going
   * Resets the Encoders
   * Resets the NavX
   * @param forwardSpeed sets the forward speed we should be going
   * @param angle sets the target angle the we should be going
   * @param distance sets the target distance we should be going
   */
  public void startGoToAngleDistance(double forwardSpeed, double angle, double distance, int PIDCheck){
    TargetDistance = distance;
    TargetAngle = angle;
    this.forwardSpeed = forwardSpeed;
    encoderReset();
    //UNCOMMENT CODE
    //navXReset();
    enableDrivePID();
    this.PIDCheck = PIDCheck;

  }
  /**
   * Is the PID that we use to drive the robot around
   * @param forwardSpeed sets the forward speed we should be going
   * @param angle sets the target angle the we should be going
   * @param distance sets the target distance we should be going
   */
  //UNCOMMENT FUNCTION
/*   public void setDriveAuton(double forwardSpeed, double angle, double distance){
    int negative = (distance < 0) ? -1 : 1;

    double abscurrentposition = Math.abs(encoderPosition());

    if(abscurrentposition < Math.abs(distance)){
      if(Math.abs(distance) - abscurrentposition > 24){
        setDriveToAngle(angle, (forwardSpeed));
      }
      else if(Math.abs(distance) - abscurrentposition > 4){
        setDriveToAngle(angle,negative * -0.1);
      }
      else{
        setDriveToAngle(angle, negative * -0.05);
      }
    }
    else{
      setDriveRate(0);
    }
  } */
  public void resetAngle(){
    navX.reset();
  }
  /**
   * Sets the drive to go to certain angle with forward speed
   * @param angle the target angle we should be going
   * @param forwardspeed Sets the forward speed we are going
   */
  //UNCOMMENT FUNCTION
  public void setDriveToAngle(double angle, double forwardspeed){
    
    double error =  angle - navX.getAngle();
    double absError = Math.abs(error);
    int negative;
    System.out.println(navX.getAngle());
    if(error < 0){
      negative = -1;
    }
    else{
      negative = 1;
    }
    if(absError > 15){
    double rate = -4 * error;
    setArcDriveRate(rate, forwardspeed);
    System.out.println(error);

    } 
    else if(absError < 15 && absError > 10){
      double rate = -150;
      setArcDriveRate(negative * rate, forwardspeed);
      } 
    else if(absError < 0.3){
      setArcDriveRate(0, forwardspeed);
    }
    else{
      double rate = -60;
      setArcDriveRate(negative * rate, forwardspeed);
    }
  }
  /**
   * Sets the rate of the arc aswell as the speeds
   * @param rate taret rate we should be going
   * @param forwardSpeed sets the forward speed we should be going
   */
  //UNCOMMENT FUNCTION
   public void setArcDriveRate(double rate, double forwardSpeed){
    double speed = drivePidController.calculate(navX.getRawGyroX(), rate); //calc the speed
    setSpeed(-speed + forwardSpeed, speed + forwardSpeed);
  } 
  public void setAngularRate(double rate, double forward){
    setArcDriveRate(rate/0.563703, forwardSpeed);
  }
  public void adjustToTarget(){
    startGoToAngleDistance(0, cameraAngle, 0, 2);
  }
  public void SmartDashboard(){
    SmartDashboard.putNumber("Angle", (double) navX.getAngle());
    SmartDashboard.putNumber("rate angle",  navX.getRawGyroZ());
    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putNumber("Camera Angle", cameraCalcAngle);

    //SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
    //SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
    //SmartDashboard.putNumber("Right current", mRightMaster.getOutputCurrent());
    //SmartDashboard.putNumber("Left current", mRightMaster.getOutputCurrent());
    //UNCOMMENT CODE
    //SmartDashboard.putNumber("Angle", navX.getRawGyroX());

  }
  @Override
  // This method will be called once per scheduler run
  
  public void periodic() {
    SmartDashboard();
    if(anglePID){
      setDriveToAngle(targetAngle, targetSpeed);
    }
    
    //UNCOMMENT FUNCTION
    /* if(isDrivePIDEnabled){
      if(PIDCheck == 1){
        drive.setDriveAuton(forwardSpeed, TargetAngle, TargetDistance);
        if(checkDistance()){
          isDrivePIDEnabled = false;
        }
      }
      else if(PIDCheck == 2){
        setDriveToAngle(TargetAngle, 0);
        if(checkAngle()){
          isDrivePIDEnabled = false;
        }
      }
      else if(PIDCheck == 3){
        drive.setDriveAuton(forwardSpeed, TargetAngle, TargetDistance);
        if(checkAngle() && checkDistance()){
          isDrivePIDEnabled = false;
        }
      }
  } */
    
  //cameraAngle = (camera.calcAngle(camera.getCenterX()) +  navX.getAngle());
  //Timer time = new Timer();
    //time.start();
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("driveTimer",  time.get());
  }

  /**
   * Enables the Drive PID
   */
  public void enableDrivePID(){
    isDrivePIDEnabled = true;
  }
  /**
   * Disables the drive PID
   */
  public void disableDrivePID(){
    isDrivePIDEnabled = false;
  }
  /**
   * gets the angle of the NavX
   * @return the angle of the NavX
   */
  //UNCOMMNET FUNCTION
/*   public double getAngle(){
    return navX.getAngle();
  } */
  /**
   * Resets the NavX
   */
  //UNCOMMENT FUNCTION
  /* public void navXReset(){
    navX.reset();
  } */
  /**
   * Sets the speed that the motors are going
   * @param left left motor speed
   * @param right right motor speed
   */
  public void setSpeed(double left, double right){
    mLeftMaster.set(-left*0.7);
    mRightMaster.set(right*0.7);
  }
  
  /** 
   * @return double
   */
  public double encoderPosition(){
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
  }
  /**
   * Resets the encoders and the current distance wew are going
   */
  public void encoderReset(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
}
