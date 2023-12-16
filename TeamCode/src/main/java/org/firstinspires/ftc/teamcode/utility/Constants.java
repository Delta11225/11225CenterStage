package org.firstinspires.ftc.teamcode.utility;

/**
 * This is a class that holds all necessary constants for the robot. Put "magic numbers" here.
 * This class cannot be instantiated. Do not inherit from it.
 * To add a new constant, simply declare a new public final static variable and assign the
 * appropriate value.
 */
public abstract class Constants {
    public final static double dumpPosition = 0.0;
    public final static double collectPosition = 0.73;
    public final static double holdPosition = 0.35;

    public final static double tseArmInitPosition = 0.47;
    public final static double tseArmCollectPosition = 0.83;
//    public final static double tseArmActivePosition = 0.46;
//
//    public final static double tseArmCollectPosition = 0.40;
//    public final static double tseArmDeployPosition = 0.2;
//    public final static double tseArmReleasePosition = 0.3;

    public final static double fastMultiplier = 1.0;
    public final static double normalMultiplier = 0.6;
    public final static double slowMultiplier = 0.25;

    public final static double superSlowMultiplier = 0.1;



    public final static double armCollectPosition = 0.85;
    public final static double armScoringPosition = 0.55;
    public final static double armHoldPosition = 0.88;



    public final static double maxLinearSlidePostion = 4100;
    public final static double linearSlideUpPower = 0.75;
    public final static double minLinearSlidePosition = 0;
    public final static double linearSlideDownPower = -0.75;
    public final static int linearSlideAutomatedDeployLow = 3000;//2771 old value
    public final static int linearSlideAutomatedDeployHigh = 4109;

    public final static double clampClosedPosition = 0.25;
    public final static double clampOpenPosition = 0.0;


    public final static int scissorHookHeightLeft = 40000;
    public final static int scissorHookHeightRight = 36000;

    public final static int scissorLiftHeightLeft = scissorHookHeightLeft-19500;

    public final static int scissorLiftHeightRight = scissorHookHeightRight-19500;

    public final static double armTrussHeight = 1;

}