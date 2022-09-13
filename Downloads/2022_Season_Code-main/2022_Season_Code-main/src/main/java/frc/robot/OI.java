package frc.robot;

import frc.robot.Auton.shoot;
//Subsystems
import frc.robot.SubSystems.Climb;
import frc.robot.SubSystems.Drive;
import frc.robot.SubSystems.Hood;
import frc.robot.SubSystems.Index;
import frc.robot.SubSystems.Intake;
import frc.robot.SubSystems.Lime;
import frc.robot.SubSystems.MegaShooter2PointO;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI extends SubsystemBase {
    // Classes
    private Climb climb;
    private Intake intake;
    private Drive drive;
    public Hood hood;
    private Index index;
    public Joystick driverControl;
    public Joystick operatorControl;
    public Lime lime;
    public MegaShooter2PointO megashooter2pointo;
    public double shootOffset = 0;

    public OI() {
        // camera = Camera.get_Instance();
        drive = Drive.get_Instance();
        driverControl = new Joystick(Constants.driverControl);
        operatorControl = new Joystick(Constants.operatorControl);
        lime = new Lime();

        // COMMENT TEST
        megashooter2pointo = MegaShooter2PointO.get_Instance();

    }

    private void driveTest() {
        if (driverControl.getRawButton(1)) {
            drive.backLeft.setAngleSpeed(0.3);
        } else {
            drive.backLeft.setAngleSpeed(0);
        }
        if (driverControl.getRawButton(2)) {
            drive.backLeft.setDriveSpeed(0.3);
        } else {
            drive.backLeft.setDriveSpeed(0);

        }
        if (driverControl.getRawButton(3)) {
            megashooter2pointo.hood.setSpeed(0.3, 0.1);
        } else {
            megashooter2pointo.hood.setSpeed(0, 0);
        }
        if (driverControl.getRawButton(4)) {
            megashooter2pointo.climb.setPositionHook(1);
        }
        if (driverControl.getRawButton(5)) {
            megashooter2pointo.climb.setPositionHook(0);
        }
        if (driverControl.getRawButton(6)) {
            megashooter2pointo.climb.setPositionArm(1);
        }
        if (driverControl.getRawButton(7)) {
            megashooter2pointo.climb.setPositionArm(0);
        }
    }

    private void driveTest2() {
        if (driverControl.getRawButton(1)) {
            megashooter2pointo.index.setSpeed(0.35);
        }

        else if (driverControl.getRawButton(2)) {
            megashooter2pointo.index.setSpeed(-0.35);
        } else {
            megashooter2pointo.index.setSpeed(0);

        }
        if (driverControl.getRawButton(3)) {
            megashooter2pointo.intake.setSpeed(0.35);
        }

        else if (driverControl.getRawButton(4)) {
            megashooter2pointo.intake.setSpeed(-0.35);
        } else {
            megashooter2pointo.intake.setSpeed(0);

        }
        if (driverControl.getRawButton(5)) {
            megashooter2pointo.intakeDown();
        }
        if (driverControl.getRawButton(6)) {
            megashooter2pointo.intakeUp();
        }

        if (driverControl.getRawButton(8)) {
            drive.frontLeft.setDriveSpeed(0.1);
        } else {
            drive.frontLeft.setDriveSpeed(0);
        }
    }

    public double driveAngle(double x, double y) {
        double done = 0;
        y = -y;
        if (x < 0) {
            if (y > 0) {
                done = (180 / Math.PI) * (3 * Math.PI / 2 - Math.atan(y / -x));
            } else {
                done = (180 / Math.PI) * (3 * Math.PI / 2 + Math.atan(-y / -x));
            }
        } else if (x > 0) {
            if (y > 0) {
                done = (180 / Math.PI) * (Math.PI / 2 + Math.atan(y / x));
            } else {
                done = (180 / Math.PI) * (Math.PI / 2 - Math.atan(-y / x));
            }
        } else {
            if (y < 0) {
                done = 1;
            } else {
                done = 180;
            }

        }

        done = Math.abs(done - 360);
        return done;
    }
    public void oiAutonStuff(){
        drive.setManual(0, 0);
    }
    public void oiRun() {
        // PERIODICS
        drive.periodicTele();
        SmartDashboard.putNumber("distance", lime.calcDistance());
        megashooter2pointo.periodic();
        // Drive
        /*
         * if(driverControl.getRawButton(1)){
         * drive.setVector(0,0,0);
         * }
         * else
         */if (cameraTracking()) {
            drive.setVector(driveAngle(driverControl.getRawAxis(0), driverControl.getRawAxis(1)),
                    Math.sqrt(Math.pow(Math.abs(driverControl.getRawAxis(0)), 2)
                            + Math.pow(Math.abs(driverControl.getRawAxis(1)), 2)) * 1.5,
                    -0.1 * lime.getHorOffset());
        } else {
            drive.setVector(driveAngle(driverControl.getRawAxis(0), driverControl.getRawAxis(1)),
                    Math.sqrt(Math.pow(Math.abs(driverControl.getRawAxis(0)), 2)
                            + Math.pow(Math.abs(driverControl.getRawAxis(1)), 2)),
                    driverControl.getRawAxis(4) * 2);
        }

        // System.out.println(lime.getHorOffset());

        if (shoot()) {
            megashooter2pointo.shoot(Constants.edgeTarmacRPM + shootOffset);
            // megashooter2pointo.shootCamera();
        } else if (shootLow()) {
            megashooter2pointo.shoot(Constants.lowGoalRPM);
        } else if (highShoot()) {
            megashooter2pointo.shoot(11500);
        } else if (extraHighLow()) {
            megashooter2pointo.shoot(10500);
        } else if (rampUp()) {
            megashooter2pointo.setShooterRPM(11500, 11500);
        } else {
            megashooter2pointo.shootOff();
        }

        // Fli heading
        if (flipHeading()) {
            megashooter2pointo.climb.setWinchSpeed(0.2, 0.2);
        }

        if (overrideIndex()) {
            megashooter2pointo.overrideIndexTrue();
        } else {
            megashooter2pointo.overrideIndexFalse();
        }
        // INDEX AND INTAKE
        if (reverseIntake()) {
            megashooter2pointo.isIndexReversed(true);
        } else if (intake()) {
            megashooter2pointo.isIndexReversed(false);
            megashooter2pointo.intakeOn();
        } else {
            megashooter2pointo.intakeOff();
            megashooter2pointo.isIndexReversed(false);
        }

        if(hooksUp()) {
            System.out.println("daijwodjaowjdajwodjaowijd");
            megashooter2pointo.hooksUp();
        }

        if (climbUp()) {
            megashooter2pointo.climb.setPositionArm(1);
        } else if (climbDown()) {
            megashooter2pointo.climb.setPositionArm(0);
        }
        if (intakeUp()) {
            megashooter2pointo.intakeUp();
        } else if (intakeDown()) {
            megashooter2pointo.intakeDown();
        }
        // CLIMB
        if (armsUp()) {
            megashooter2pointo.enableArmsUp();
        } else if (climbToLvl1()) {
            megashooter2pointo.enableTravClimblvl1();
        } else if (climbToLvl2()) {
            megashooter2pointo.enableTravClimblvl2();
        } else if (climbToLvl3()) {
            megashooter2pointo.enableTravClimblvl3();
        } else if (resetClimb()) {
            megashooter2pointo.enableReset();
        } else if (takeOffClimb()) {
            megashooter2pointo.enableTakeOff();
        }
        if(autoClimbTest()){
            megashooter2pointo.initAutoClimb();
        }
        if(disableAutoClimbTest()){
            megashooter2pointo.disableClimb();
        }

        if(driverControl.getRawButton(5)){
            shootOffset += 50;
        }else if(driverControl.getRawButton(6)){
            shootOffset -= 50;
        }
        SmartDashboard.putNumber("shooter + offset", Constants.edgeTarmacRPM + shootOffset);
        // TEST OI

        // GITHUB??????
        // GITHUB 2

    }

    // DRIVER CONTROLS
    public boolean homeSwerve() {
        return driverControl.getRawButton(1);
    }

    public boolean cameraTracking() {
        return driverControl.getRawButton(2);
    }

    public boolean flipHeading() {
        return driverControl.getRawButton(7);
    }

    public boolean intake() {
        return driverControl.getRawAxis(3) > 0.3 || operatorControl.getRawAxis(2) > 0.3;
    }
    public boolean autoClimbTest(){
        return driverControl.getRawButton(4);
    }
    public boolean disableAutoClimbTest(){
        return driverControl.getRawButton(3);
    }

    // POV
    public boolean highShoot() {
        return driverControl.getPOV() == 0;
    }

    public boolean extraHighLow() {
        return driverControl.getPOV() == 180;
    }

    public boolean takeOffClimb() {
        return driverControl.getPOV() == 270;
    }

    // OPERATORS
    public boolean shoot() {
        return operatorControl.getRawAxis(3) > 0.3;
    }

    public boolean armsUp() {
        return operatorControl.getRawButton(3);
    }

    public boolean climbToLvl1() {
        return operatorControl.getRawButton(1);
    }

    public boolean climbToLvl2() {
        return operatorControl.getRawButton(2);
    }

    public boolean climbToLvl3() {
        return operatorControl.getRawButton(4);
    }

    public boolean resetClimb() {
        return operatorControl.getRawButton(8);
    }

    public boolean rampUp() {
        return operatorControl.getRawButton(6);
    }

    public boolean reverseIntake() {

        return operatorControl.getRawButton(5);
    }

    public boolean shootLow() {
        return operatorControl.getRawButton(7);
    }

    public boolean overrideIndex() {
        return operatorControl.getRawButton(10);
    }

    public boolean hooksUp() {
        return operatorControl.getRawButton(9);
    }

    // POV
    public boolean climbUp() {
        return operatorControl.getPOV(0) == 270;
    }

    public boolean climbDown() {
        return operatorControl.getPOV(0) == 90;
    }

    public boolean intakeDown() {
        return operatorControl.getPOV(0) == 180;
    }

    public boolean intakeUp() {
        return operatorControl.getPOV(0) == 0;
    }

}