package org.firstinspires.ftc.teamcode.models;

public class MiscMethods {
    public static float angleWrap(float angle){
        if (angle < -Math.PI)
            angle += 2 * Math.PI;
        else if (angle > Math.PI)
            angle -= 2 * Math.PI;
        return angle;
    }
}
