package frc.robot.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Trapezoid{
    private double PStart;
    private double PEnd;
    private double VStart;
    private double ALead;
    private double ATail;
    private double VMax;
    private double VMin;
    private double VStartMinPos;
    private double tolerance;
    private double VStartMinNeg;
    private double minGoingRateNeg;
    private double minGoingRatePos;
    
    public Trapezoid(double minGoingRatePos, double minGoingRateNeg, double tolerance, double VStartMinPos, double VStartMinNeg, double VMin, double VMax, double PStart, double PEnd, double VStart, double ALead, double ATail){
        this.PStart = PStart;
        this.PEnd = PEnd;
        this.VStart = VStart;
        this.ALead = ALead;
        this.ATail = ATail;
        this.VMin = VMin;
        this.VMax = VMax;
        this.VStartMinPos = VStartMinPos;
        this.VStartMinNeg = VStartMinNeg;
        this.minGoingRateNeg = minGoingRateNeg;
        this.minGoingRatePos = minGoingRatePos;
        
        this.tolerance = tolerance;
    }
    
    /** 
     * @param PStart
     * @param PEnd
     * @param VStart
     */
    public void trapezoidUpdate(double PStart, double PEnd, double VStart){
        this.PStart = PStart;
        this.PEnd = PEnd;
        this.VStart = VStart;
        
    }
    /**
     * calculate velocity output
     * @param PRaw raw value
     * @return calculaed velocity
     */
    public double trapezoidCalc(double PRaw, double PRate){
        double VOut;
        double PTotal = PEnd - PStart;              // Total travel distance
        double PCur = PRaw - PStart;                // Distance from start position
        double PErr = PTotal - PCur;                // Distance to target
        double VLead = ALead * PCur;                // Leadin speed
        double VTail = ATail * PErr;                // Tail speed

        // Find the speed that is closest to 0
        if(Math.abs(VLead) < Math.abs(VTail)){
            if(PErr > 0) {
                VLead = Math.max(VLead, minGoingRatePos);
            } else{
                VLead = Math.min(VLead, minGoingRateNeg);
            }
            VOut = VLead;
        }
        else{
            VOut = VTail;
        }
        if(Math.abs(VOut) < Math.abs(minGoingRatePos)){
            VOut = (PRaw > PEnd) ? (minGoingRateNeg) : (minGoingRatePos);
        }
        // Ensure speed is within limits
        if(Math.abs(PRate)< 15){
            VOut = (PRaw > PEnd) ? (VStartMinNeg) : (VStartMinPos);
        }
        VOut = MathUtil.clamp(VOut, VMin, VMax);

        // Stop moing if the error is outside of 
        if(Math.abs(PErr) < tolerance){
            VOut = 0;
        }
        
        return VOut;
        
    }
}