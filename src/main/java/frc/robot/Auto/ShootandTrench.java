package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.*;

public class ShootandTrench extends SequentialCommandGroup{
    public ShootandTrench(){
        
        new AdjustToTarget();
        //shoot not done yet

        new gotoAngle(0);
        new MoveDistance(-285, 0, 0.2);

        //parallel intake not done yet
        new AdjustToTarget();
        //shoot not done yet


        
    }
}