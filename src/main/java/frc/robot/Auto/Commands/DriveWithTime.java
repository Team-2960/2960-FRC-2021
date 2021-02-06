package frc.robot.Auto.Commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;

public class DriveWithTime extends CommandBase{
    private Drive drive = Drive.get_Instance();
    private Timer timer = new Timer();
    private double left , right;
    private double time;
    private boolean isFinish;

    public DriveWithTime(double left, double right, double time){
        this.left = left;
        this.right = right;
        this.time = time;
    }
    public DriveWithTime(double speed, double time){
        this.left = speed;
        this.right = speed;
        this.time = time;
    }
@Override
    public void initialize() {
        super.initialize();
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
        isFinish = timer.get() > time;
        drive.setSpeed(left, right);
    }

    
    /** 
     * @param interrupt
     */
    @Override
    public void end(boolean interrupt) {
        drive.setSpeed(0,0);
        drive.disableDrivePID();

    }
}
