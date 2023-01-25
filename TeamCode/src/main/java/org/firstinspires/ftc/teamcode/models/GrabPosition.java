package org.firstinspires.ftc.teamcode.models;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;

public class GrabPosition {
    public int verticalPos = ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
    public double armPos;
    public int horizontalPos;

    public GrabPosition(double armPos, int horizontalPos) {
        this.armPos = armPos;
        this.horizontalPos = horizontalPos;
    }

    public GrabPosition(double armPos, int horizontalPos, int verticalPos) {
        this.armPos = armPos;
        this.horizontalPos = horizontalPos;
        this.verticalPos = verticalPos;
    }
}
