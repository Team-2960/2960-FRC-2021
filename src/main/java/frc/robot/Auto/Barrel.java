package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Commands.*;

public class Barrel extends SequentialCommandGroup{
    public Barrel(){
        addCommands(
            //new DriveAngle(1, 90, 0.75),
            new DriveWithTime(-1, -1, 1)
        );



    }
}