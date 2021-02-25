package frc.robot.Auto.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import edu.wpi.first.wpilibj.Timer;


public class DriveAngle extends CommandBase{
    //shoot the ball
    
    Drive drive = Drive.get_Instance();
    
    private boolean isFinish = false;
    double timeSec;
    double angle;
    double fps;
    Timer timer = new Timer();
    public DriveAngle(double timeSec, double angle, double fps){
        this.timeSec = timeSec;
        this.angle = angle;
        this.fps = fps;
    }

    @Override
    public void initialize() {
        super.initialize();
        drive.setAngularRate(angle,fps);
        timer.start();
    }

    /**
     * Returns true if all the commands in this group have been started and have
     * finished.
     * <p>
     * <p>
     * Teams may override this method, although they should probably reference
     * super.isFinished() if they do.
     * </p>
     *
     * @return whether this {@link CommandGroup} is finished
     */
    @Override
    public boolean isFinished() {
        if (isFinish)
            return true;
        else
            return false;
    }

    @Override
    public void execute() {
        isFinish = timer.get() > timeSec;
    }

    
    /** 
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        drive.setAngularRate(0,0);
        timer.stop();
    }
}