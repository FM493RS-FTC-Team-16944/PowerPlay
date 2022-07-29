package org.firstinspires.ftc.teamcode.gamepad.easypad;

import java.util.Arrays;
import java.util.List;

public enum Mapping {
    LEFT_STICK_X("left_stick_x"),
    LEFT_STICK_Y("left_stick_y"),
    RIGHT_STICK_X("right_stick_x"),
    RIGHT_STICK_Y("right_stick_y"),
    MOVEMENT(Arrays.asList("left_stick_x", "left_stick_y", "right_stick_x", "right_stick_y")),
    DPAD_UP("dpad_up"),
    DPAD_DOWN("dpad_down"),
    DPAD_LEFT("dpad_left"),
    DPAD_RIGHT("dpad_right"),
    BUTTON_A("a"),
    BUTTON_B("b"),
    BUTTON_X("x"),
    BUTTON_Y("y"),
    BUTTON_GUIDE("guide"),
    BUTTON_START("start"),
    BUTTON_BACK("back"),
    LEFT_BUMPER("left_bumper"),
    RIGHT_BUMPER("right_bumper"),
    LEFT_STICK_BUTTON("left_stick_button"),
    RIGHT_STICK_BUTTON("right_stick_button"),
    LEFT_TRIGGER("left_trigger"),
    RIGHT_TRIGGER("right_trigger");

    public Object map;

    Mapping(String map) {
        this.map = map;
    }

    Mapping(List<String> map) {
        this.map = map;
    }
}
