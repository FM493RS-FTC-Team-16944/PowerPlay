package org.firstinspires.ftc.teamcode.models;

public class MiscMethods {
    public static double angleWrap(double angle){
        if (angle < -Math.PI)
            angle += 2 * Math.PI;
        else if (angle > Math.PI)
            angle -= 2 * Math.PI;
        return angle;
    }
}
