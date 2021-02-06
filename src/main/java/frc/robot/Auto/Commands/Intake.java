package frc.robot.Auto.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubSystems.Shooter;

public class Intake extends CommandBase{
    //shoot the ball
    
    Shooter shooter = Shooter.get_Instance();
    private boolean isFinish = false;

    public Intake(){
    }

    @Override
    public void initialize() {
        super.initialize();
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
    }

    
    /** 
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
    }
}