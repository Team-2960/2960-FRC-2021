package frc.robot.Auto.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;

public class gotoAngle extends CommandBase {
    //go to a angle;
    Drive drive = Drive.get_Instance();
    private boolean isFinish = false;
    private double angle;
    /**
     * go to the angle
     * @param angle target angle
     */
    public gotoAngle(double angle){
        this.angle = angle;
    }

    @Override
    public void initialize() {
        super.initialize();
        drive.startGoToAngleDistance(0, angle, 0, 2);
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
        //UNCOMMENT CODE
        //isFinish = drive.checkAngle();
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