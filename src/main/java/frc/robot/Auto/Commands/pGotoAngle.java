package frc.robot.Auto.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Drive;
import frc.robot.SubSystems.Pivot;

public class pGotoAngle extends CommandBase{
    //this will adjust the shooting angle

    Pivot pivot = Pivot.get_Instance();
    private boolean isFinish = false;
    private double angle;

    public pGotoAngle(double angle){
        this.angle = angle;
    }

    @Override
    public void initialize() {
        super.initialize();
        pivot.setPTargetAngle(angle);
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
        isFinish = pivot.atPivotTarget();
    }

    
    /** 
     * @param interrupt
     */
    @Override
    public void end(boolean interrupt) {
        //WILL CHANGE TO THIS NAME LATER
    }
}