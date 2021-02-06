package frc.robot.Auto.Commands;

import edu.wpi.first.wpilibj2.command. CommandBase;
import edu.wpi.first.wpilibj.Timer;
public  class Delay extends CommandBase{
    //shoot the ball
    private Timer timer;
    private double delay;
 
    private boolean isFinish = false;
   

	
    public Delay(double delayTime){
        delay = delayTime;
    }

    @Override
    public void initialize() {
        super.initialize();
        timer = new Timer();
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
        isFinish = timer.get() >= delay;
    }

    
    /** 
     * @param interrupte
     */
    @Override
    public void end(boolean interrupte) {
        timer.stop();
    }
}