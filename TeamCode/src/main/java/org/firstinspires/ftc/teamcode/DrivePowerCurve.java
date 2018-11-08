package org.firstinspires.ftc.teamcode;

public class DrivePowerCurve {

    public static double threePartPoly(double stickPositon){
        return (Math.pow(stickPositon,0.33333333) + Math.pow(stickPositon,3) + Math.pow(stickPositon,5)) /3;
    }

    public static float valueSquared(float stickPosition) {
        return Math.abs(stickPosition) * stickPosition;
    }

    public static float linear(float stickPosition){
        return stickPosition;
    }

    public enum MODE {LINEAR, SQUARED,POLYNOMIAL}
}
