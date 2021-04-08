package frc.robot;
public class Constants{
    /**
     * Joystick
     */
    public final static int driver_Control_Left = 0;
    public final static int driver_Control_Right = 1;
    public final static int operator_Control = 2;
    public final static int backUp_Control = 3;
    /**
     * Drive train motor
     */
    public final static int mLeftMaster1 = 1;
    public final static int mLeftFollow2 = 2;
    public final static int mLeftFollow3 = 3;
    public final static int mRightMaster1 = 4;
    public final static int mRightMfollow2 = 5;
    public final static int mRightMfollow3 = 6;

    /**
     * Shooter motor
     */
    public final static int mLeftShooter = 7;
    public final static int mRightShooter = 8;
    
    /**
     * Pivot Motor
     */
    public final static int mLeftPivot = 9;
    public final static int mRightPivot = 10;

    /**
     * Intake motor
     */
    public final static int mIntake = 11;

    /**
     * Climb Motor
     */
    public final static int mClimb = 12;
    public final static int mClimb2 = 20;


    /**
     * Index Motor
     */
    public final static int mIndex1 = 13;
    public final static int mIndex2 = 14;

    /**
     * Pivot PID control
     */
    public final static double pKp = 0.00024;
    public final static double pKi = 0.0000;
    public final static double pKd = 0.0000000000000;
    /**
     * The feedforward PID
     */
    public final static double pKs = 0.988;
    public final static double pKcos = 0.0148;
    public final static double pKv = 0.0268;
    public final static double pKa = 0.00261;
    /**
     * The PID for the rate on the drive train
    */
    public final static double dKp = 0.001;//used to be 0.003
    public final static double dKi = 0.00000/*7*/;
    public final static double dKd = 0.0000/*25*/;
    

    /**
     * camera
     */
    public final static int cWidth = 640;
    public final static int cHeight = 480;
    /**
     * values for grippipelines
     */
    public static double[] hsvThresholdHue = {50, 90};
    public static double[] hsvThresholdSaturation = {80, 240};
    public static double[] hsvThresholdValue = {130, 265};
    /**
     * view angles
     */
    public final static double horizontalViewAngle = 61;
    public final static double verticalViewAngle = 20.55;


    public final static int IntakeSolenoid1 = 0;
    public final static int IntakeSolenoid2 = 1;
    public final static int ClimbSolenoid1 = 2;
    public final static int ClimbSolenoid2 = 3;

    public final static int cameraPort = 0;

    public final static int pabsEncoder = 0;
    public final static int pEncoder1 = 1;
    public final static int pEncoder2 = 2;
    public final static int shooterTolerance = 200;


    /**
     * The degrees per pixel on the Microsoft Life Cam HD -3000
     */
    public final static double deg_per_px = verticalViewAngle / cHeight;
    /**
     * The diameter of the drive train wheels
     */
    public final static double wheelDiam = 6;
    /**
     * the pulses per revolution on the drive train encoders
     */
    public final static int pulsePerRev = 8192;
    /**
     * The Distance per pulses
     */
    public final static double wheelcircumference = (wheelDiam * Math.PI);
    /**
     * The Tolerance for target distance
     */
    public final static double distanceTolerance = 2;
    /**
     * The tolerance for the target angle
     */
    public final static double angleTolerance = 3;
    /**
     * The front neutural angle
     */
    public final static double neuturalPosFront = 230;
    /**
     * The front neutural angle
     */
    public final static double neuturalPosBack = 160;
    /**
     * Neutural Pos window
     */
    public final static double frontWindowMin = 20;
    public final static double frontWindowMax = 180;
    public final static double backWindowMin = 200;
    public final static double backWindowMax = 270;
    /**
     * Pivot angle and speed table in feet
     * first is distance 
     * second is angle
     * third is speed
     */
/*     public final static double pivotTable [] [] = {
                                                    {1, 180, 180, 4000},
                                                    {1.5, 160, 160, 4000},
                                                    {2, 140, 140, 4000},
                                                    {2.5, 120, 120, 4000},
                                                    {3, 100, 100, 4000},
                                                    {3.5, 80, 80, 4000},
                                                    {4, 60, 60, 4000},
                                                    {4.5, 40, 40, 4000},
                                                    {5, 20, 20, 4000}
                                                }; */
    public final static double longPreset [] ={-6500, 149, 150};//1-2
    public final static double shortPreset [] ={-4000, 179, 150};
    public final static double preset2 [] = {-7500, 144,150};//2-3
    public final static double preset3 [] = {-8000, 140 ,150};//3-4
    public final static double preset4 [] = {-6000, 140 ,150};//{-5000, 170 ,150};//0-1
    public final static double autonPreset [] ={-6500, 170, 100};
    public final static double feederPreset [] = {2000, 290, 100};
    public final static double wheelOfFortunePreset [] = { 1000, 150, 2960};

    public final static double intakeSpeedIn = 0.4;
    public final static double intakeSpeedOut = -0.4;
    public final static double intakePivotAngle = 30;
    public final static double intakeShooterSpeed = -500;
    public final static double percentOnLowerBelt = 0.85;
    public final static double pivotOutOfReach = 260;
}