package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {

    public static double verticalKP = 0.02;
    public static double verticalKI = 0.01;
    public static double verticalKD = 0.00;

    public static double horizontalKP = 0.014;
    public static double horizontalKI = 0.001;
    public static double horizontalKD = 0.00;

    public static double HORIZONTAL_SLIDE_POWER = 0.60;
    public static double VERTICAL_LIFT_POWER = 0.60;

    public static double OPEN_CLAW_POSITION = 0.70;
    public static double CLOSE_CLAW_POSITION = 1.00;

    public static double ARM_CLAW_POSITION_FIRST_CONE = 0.40;
    public static double ARM_CLAW_POSITION_SECOND_CONE = 0.35;
    public static double ARM_CLAW_POSITION_THIRD_CONE = 0.27;
    public static double ARM_CLAW_POSITION_FOURTH_CONE = 0.20;
    public static double ARM_CLAW_POSITION_FIFTH_CONE = 0.00;
    public static double ARM_CLAW_POSITION_NEUTRAL = 0.20;

    public static double SECOND_PART_UP_ARM_CLAW_POSITION_FIRST_CONE = 0.00;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_SECOND_CONE = 0.00;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_THIRD_CONE = 0.00;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_FOURTH_CONE = 0.20;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_FIFTH_CONE = 0.00;

    public static double DROP_ARM_CLAW_POSITION = 0.25;

    public static double NEUTRAL_CLAW_TILT_POSITION = 1.00;
    public static double UP_CLAW_TILT_POSITION = 0.30;
    public static double DROP_CLAW_TILT_POSITION = 0.00; // this is the tilt when it's gonna drop it on the red thing

    public static double NORMAL_ROTATOR_POSITION = 0.25;
    public static double ONE_EIGHTY_ROTATOR_POSITION = 1.00;

    public static int NEUTRAL_VERTICAL_LIFT_POSITION = 0;
    public static int HIGH_SCORE_VERTICAL_LIFT_POSITION = 2340;
    public static int MEDIUM_SCORE_VERTICAL_LIFT_POSITION = 1190;
    public static int LOW_SCORE_VERTICAL_LIFT_POSITION = 520;

    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE = 2200;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE = 2400;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE = 2500;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE = 2600;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE = 2700;
    public static int HORIZONTAL_SLIDE_NEUTRAL_POSITION = 0;
}
