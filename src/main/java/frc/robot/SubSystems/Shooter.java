package frc.robot.SubSystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class Shooter extends SubsystemBase{
    private static Shooter shooter;
    
    //shooter motor
    private TalonFX mLeftShooter;
    private TalonFX mRightShooter;

    private double targetRate = 0;
    private double speedOffset;


    //pid value will move to constants later.
    //right
    public double kp1 = 0.003,
                  ki1 = 0.0000/*46*/,
                  kd1 = 0.0; 
    //left
    public double kp2 = 0.008, /*0.05 0.525 very close to 0*/
                  ki2 = 0.0000/*46*/,              
                  kd2 = 0.0;              
    /** 
     * @return Shooter\][
     * 
     * ]
     */
    public static Shooter get_Instance(){
      if(shooter == null){
        shooter = new Shooter();
      } 
      return shooter;
    }
    
    private Shooter(){
      //init code
      mLeftShooter = new TalonFX(Constants.mLeftShooter);
      mRightShooter = new TalonFX(Constants.mRightShooter);

      
      //invert right shooter motor
      mRightShooter.setInverted(true);

      //set the rigth motor pid
      mRightShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
      
      mRightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
      mRightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
      //out put range
      mRightShooter.configNominalOutputForward(0, 30);
      mRightShooter.configNominalOutputReverse(0, 30);
      mRightShooter.configPeakOutputForward(1, 30);
      mRightShooter.configPeakOutputReverse(-1, 30);
      //set PID value
      mRightShooter.selectProfileSlot(0, 0);
		  mRightShooter.config_kP(0, kp1, 30);
		  mRightShooter.config_kI(0, ki1, 30);
      mRightShooter.config_kD(0, kd1, 30);
      mRightShooter.config_kF(0, 1023.0/21700.0, 30);
      
      //get sensor value
      mLeftShooter.setSelectedSensorPosition(0, 0, 30);
      
      mLeftShooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
      
      mLeftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
      mLeftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

      mLeftShooter.configNominalOutputForward(0, 30);
      mLeftShooter.configNominalOutputReverse(0, 30);
      mLeftShooter.configPeakOutputForward(1, 30);
      mLeftShooter.configPeakOutputReverse(-1, 30);

      mLeftShooter.selectProfileSlot(0, 0);
		  mLeftShooter.config_kP(0, kp2, 30);
		  mLeftShooter.config_kI(0, ki2, 30);
      mLeftShooter.config_kD(0, kd2,30);
      mLeftShooter.config_kF(0, 1023.0/20500, 30);
      
      mLeftShooter.setSelectedSensorPosition(0, 0, 30);
    }
    /**
     * Sets the Shooter Speed 
     * @param speed the speed you want the shooter to be set to
     */
    public void setShooterSpeed(double right, double left){
      mRightShooter.set(ControlMode.PercentOutput, -right);
      mLeftShooter.set(ControlMode.PercentOutput, -left);
    }
    
    
    /** 
     * @param speed
     */
    public void setSpeedOffset(double speed){
      speedOffset = 10000 * speed;
    }
    
    /** 
     * @param rate
     */
    public void gotoRate(double rate){
      targetRate = rate /* + speedOffset*/;
      setPIDShooterSpeed(targetRate);
    }

    /** 
     * @return boolean
     */
    public boolean readyToShoot(){
      boolean readyToShoot = false;
      double rError = Math.abs(Math.abs(mRightShooter.getSelectedSensorVelocity()) - Math.abs(targetRate));
      double lError = Math.abs(Math.abs(mLeftShooter.getSelectedSensorVelocity()) - Math.abs(targetRate));
      if(rError < 300 || lError < 300){
        readyToShoot = true;
      }
      return readyToShoot;
    }

    public void SmartDashBoard(){
      System.out.println("left " + mLeftShooter.getSelectedSensorVelocity());
      System.out.println("right " + mRightShooter.getSelectedSensorVelocity());

      SmartDashboard.putNumber("Right shooter encoder", mRightShooter.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Left shooter encoder ", mLeftShooter.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Shooter LEft Speed", mLeftShooter.getMotorOutputPercent());
      SmartDashboard.putNumber("Shooter Right Speed", mRightShooter.getMotorOutputPercent());
      SmartDashboard.putNumber("Target Rate", targetRate);
      SmartDashboard.putBoolean("ReadyToShoot", readyToShoot());

    }
    public void periodic() {
      //Timer time = new Timer();
      //time.start();
    SmartDashBoard();
    // This method will be called once per scheduler run
      //SmartDashboard.putNumber("shooterTimer",  time.get());
    }



    /**
     * Sets the PID shooter speed 
     * @param speed the PID shooter speed
     */
    private void setPIDShooterSpeed(double speed){
      mRightShooter.set(ControlMode.Velocity, speed);
      mLeftShooter.set(ControlMode.Velocity, speed+150);
    }
}