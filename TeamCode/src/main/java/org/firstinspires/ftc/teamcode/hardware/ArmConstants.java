package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static double HORIZONTAL_SLIDE_POWER = 1.00;
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

    public static double DROP_ARM_CLAW_POSITION = 0.39;
    public static double DROP_ARM_CLAW_POSITION_B = 0.41;
    public static double DROP_ARM_CLAW_POSITION_C = 0.45;

    public static double ARM_CLAW_CONE_RETRIEVAL = 0.05;                //TODO: TUNE AF
    public static double ARM_CLAW_CONE_RETRIEVAL_UP = 0.20;             //TODO: TUNE AF
    public static double ARM_CLAW_CONE_RETRIEVAL_INVERTED = 0.07;      //TODO: TUNE AF

    public static double TILT_CLAW_CONE_RETRIEVAL = 1.00;                //TODO: TUNE AF
    public static double TILT_CLAW_CONE_RETRIEVAL_UP = 0.8;             //TODO: TUNE AF



    public static double NEUTRAL_CLAW_TILT_POSITION = 0.61;       //former 0.095
    public static double UP_CLAW_TILT_POSITION = 0.40;            // former: 0.70
    public static double DROP_CLAW_TILT_POSITION = 0.00;          // former: 0.34


    public static double NORMAL_ROTATOR_POSITION = 0.25;
    public static double ONE_EIGHTY_ROTATOR_POSITION = 0.95;

    public static int NEUTRAL_VERTICAL_LIFT_POSITION = 0;
    public static int HIGH_SCORE_VERTICAL_LIFT_POSITION = 2220;
    public static int MEDIUM_SCORE_VERTICAL_LIFT_POSITION = 1340;
    public static int LOW_SCORE_VERTICAL_LIFT_POSITION = 520;

    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE = 2394;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE = 2186;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE = 2107;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE = 2059;
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE = 1963;
    public static int HORIZONTAL_SLIDE_NEUTRAL_POSITION = 0;

    public static double LEFT_ODOM_RETRACTED = 1;
    public static double AUX_ODOM_RETRACTED = 1;
    public static double LEFT_ODOM_LOWERED = 0;
    public static double AUX_ODOM_LOWERED = 0;

    public static double ACTIVE_SLIDE_POSITION = 0;
    public static double DEACTIVE_SLIDE_POSITION = 0.1;
}
