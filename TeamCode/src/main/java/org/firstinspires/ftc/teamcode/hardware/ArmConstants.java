package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {

    public static double verticalKP = 0.027;
    public static double verticalKI = 0.00;
    public static double verticalKD = 0.00;

    public static double horizontalKP = 0.010;
    public static double horizontalKI = 0.00;
    public static double horizontalKD = 0.00;

    public static double HORIZONTAL_SLIDE_POWER = 0.60;
    public static double VERTICAL_LIFT_POWER = 1.00;

    public static double OPEN_CLAW_POSITION = 0.70;
    public static double CLOSE_CLAW_POSITION = 1.00;

    public static double ARM_CLAW_POSITION_FIRST_CONE = 0.27;
    public static double ARM_CLAW_POSITION_SECOND_CONE = 0.203;
    public static double ARM_CLAW_POSITION_THIRD_CONE = 0.15;
    public static double ARM_CLAW_POSITION_FOURTH_CONE = 0.10;
    public static double ARM_CLAW_POSITION_FIFTH_CONE = 0.00;
    public static double ARM_CLAW_POSITION_NEUTRAL = 0.20;

    public static double SECOND_PART_UP_ARM_CLAW_POSITION_FIRST_CONE = 0.00;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_SECOND_CONE = 0.00;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_THIRD_CONE = 0.00;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_FOURTH_CONE = 0.20;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_FIFTH_CONE = 0.00;

    public static double DROP_ARM_CLAW_POSITION = 0.250;

    public static double NEUTRAL_CLAW_TILT_POSITION = 0.95;
    public static double UP_CLAW_TILT_POSITION = 0.70;
    public static double DROP_CLAW_TILT_POSITION = 0.37; // this is the tilt when it's gonna drop it on the red thing

    public static double NORMAL_ROTATOR_POSITION = 0.25;
    public static double ONE_EIGHTY_ROTATOR_POSITION = 0.95;

    public static int NEUTRAL_VERTICAL_LIFT_POSITION = 0;
    public static int HIGH_SCORE_VERTICAL_LIFT_POSITION = 1390;
    public static int MEDIUM_SCORE_VERTICAL_LIFT_POSITION = 1190;
    public static int LOW_SCORE_VERTICAL_LIFT_POSITION = 520;

    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE = 1500;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE = 1370;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE = 1320;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE = 1290;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE = 1230;
    public static int HORIZONTAL_SLIDE_NEUTRAL_POSITION = 0;
}
