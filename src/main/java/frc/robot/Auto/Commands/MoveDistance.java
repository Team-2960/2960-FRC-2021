
package frc.robot.Auto.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;

public class MoveDistance extends CommandBase{

    Drive drive = Drive.get_Instance();
    private boolean isFinish = false;
    private double distance;
    private double speed;
    private double angle;

    public MoveDistance(double distance, double angle, double speed){
        this.distance = distance;
        this.speed = speed;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        super.initialize();
        drive.startGoToAngleDistance(speed, angle, distance, 1);
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
        isFinish = drive.checkDistance();
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
