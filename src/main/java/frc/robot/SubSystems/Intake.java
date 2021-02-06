/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.SubSystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static Intake intake;
  private CANSparkMax mIntake;
  private DoubleSolenoid sIntake;
  private Timer IntakeTimer = new Timer();
  
  /** 
   * @return Intake
   */
  public static Intake get_Instance(){
    if(intake == null){
      intake = new Intake();
    } 
    return intake;
  }
  private Intake() {
    sIntake = new DoubleSolenoid(Constants.IntakeSolenoid1, Constants.IntakeSolenoid2);
    mIntake = new CANSparkMax(Constants.mIntake, MotorType.kBrushless);
    IntakeTimer.start();
    setPosition(0);
  }

  
  /** 
   * @param speed
   */
  public void setSpeed(double speed){
    mIntake.set(speed);
  }
  
  /** 
   * @param setPosition
   * @return 
   */
  //not sure need to test
  public void setPosition(int state){
    if(state == 0){
      sIntake.set(Value.kForward);
    }
    else if(state == 1){
      sIntake.set(Value.kReverse);
      IntakeTimer.reset();
    }     
  }
  public boolean isIntakeOut(){
    if(DoubleSolenoid.Value.kForward == sIntake.get()){
      return false;
    }
    else{
      return true;
    }
  }
  public double getTime(){
    return IntakeTimer.get();
  }
  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
    //Timer time = new Timer();
    //time.start();
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("intakeTimer",  time.get());
  }
}