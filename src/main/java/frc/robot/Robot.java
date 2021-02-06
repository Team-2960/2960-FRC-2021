package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.Camera.Camera;
import frc.robot.OI;
import frc.robot.Auto.ShootAndMove;
import frc.robot.Constants;
import frc.robot.SubSystems.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private OI oi;
  private LEDs leds;
  private Camera camera = Camera.get_Instance();
  private Command autonCommand = null;
  @Override
  public void robotInit() {
    oi = new OI();
    leds = new LEDs();
  }

  @Override
  public void autonomousInit() {
    autonCommand = new ShootAndMove();
    if(autonCommand != null) autonCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    if(autonCommand != null)
    autonCommand.cancel();
  }

  @Override
  public void robotPeriodic() {
    // TODO Auto-generated method stub
    super.robotPeriodic();
    CommandScheduler.getInstance().run();
    //Timer time = new Timer();
    //time.start();
    //SmartDashboard.putNumber("robot Timer", time.get());
    //camera.update();
    
  }

  @Override
  public void disabledInit() {
    // TODO Auto-generated method stub
    super.disabledInit();
    //oi.pivot.DisablePivotPID();
  }

  @Override
  public void teleopPeriodic() {
    //no code have to be here
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
