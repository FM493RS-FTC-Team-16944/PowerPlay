package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static double HORIZONTAL_SLIDE_POWER = 1.00;
    public static double VERTICAL_LIFT_POWER = 1.00;

    public static double OPEN_CLAW_POSITION = 0.70;
    public static double CLOSE_CLAW_POSITION = 1.00;

    //TODO: NOTE: Arm claw positions changed as a result of switching to a 5 turn servo. As such, each of the current values are 5 turn values. Conversion Rate is (5turn value) = OG value * 0.175

    public static double ARM_CLAW_POSITION_FIRST_CONE = 0.0355; // 0.0452;           // For regular torque: 0.27
    public static double ARM_CLAW_POSITION_SECOND_CONE = 0.0255;         // For regular torque: 0.203
    public static double ARM_CLAW_POSITION_THIRD_CONE = 0.0205;           // For regular torque: 0.15
    public static double ARM_CLAW_POSITION_FOURTH_CONE = 0.01;          // For regular torque: 0.10
    public static double ARM_CLAW_POSITION_FIFTH_CONE = 0.00;           // For regular torque: 0.00
    public static double ARM_CLAW_POSITION_NEUTRAL = 0.20;              // For regular torque: 0.20


    public static double DROP_ARM_CLAW_POSITION = 0.0675;                 // For regular torque: 0.39
    public static double DROP_ARM_CLAW_POSITION_B = 0.0675;               // For regular torque: 0.41
    public static double DROP_ARM_CLAW_POSITION_C = 0.07175;               // For regular torque: 0.45

    public static double ARM_CLAW_CONE_RETRIEVAL = 0.00875;               // For regular torque: 0.05
    public static double ARM_CLAW_CONE_RETRIEVAL_UP = 0.035;            // For regular torque: 0.20
    public static double ARM_CLAW_CONE_RETRIEVAL_INVERTED = 0.01225;      // For regular torque: 0.07

    public static double TILT_CLAW_CONE_RETRIEVAL = 1.00;
    public static double TILT_CLAW_CONE_RETRIEVAL_UP = 0.8;


    public static double NEUTRAL_CLAW_TILT_POSITION = 0.61;       //former 0.095
    public static double UP_CLAW_TILT_POSITION = 0.40;            // former: 0.70
    public static double DROP_CLAW_TILT_POSITION = 0.00;          // former: 0.34


    public static double NORMAL_ROTATOR_POSITION = 0.255;
    public static double ONE_EIGHTY_ROTATOR_POSITION = 0.915; //changed to 0.39 for a 5 turn servo (old rev one was 0.95)

    public static int NEUTRAL_VERTICAL_LIFT_POSITION = 0;
    public static int HIGH_SCORE_VERTICAL_LIFT_POSITION = 2230;
    public static int MEDIUM_SCORE_VERTICAL_LIFT_POSITION = 1350;

    public static int RANGE_OF_UNSAFE_VERTICAL_LIFT = 500;

    public static int LOW_SCORE_VERTICAL_LIFT_POSITION = 570;

    public static double TILT_THRESHOLD = Math.toRadians(5);

    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE = 1750; //2394
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE = 1650; //2186
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE = 1650; //2107
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE = 1600; //2059
    public static int HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE = 1600; //1963
    public static int HORIZONTAL_SLIDE_NEUTRAL_POSITION = 0;

    public static double LEFT_ODOM_RETRACTED = 0;
    public static double AUX_ODOM_RETRACTED = 1;
    public static double LEFT_ODOM_LOWERED = 0.4;
    public static double AUX_ODOM_LOWERED = 0.6;

    public static double ACTIVE_SLIDE_POSITION = 0.2;
    public static double DEACTIVE_SLIDE_POSITION = 0;
}
