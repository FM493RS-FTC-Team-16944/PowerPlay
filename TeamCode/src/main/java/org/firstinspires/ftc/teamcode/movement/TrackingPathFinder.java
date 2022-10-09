package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Objects;

public class TrackingPathFinder {
    private final LinearOpMode teleOp;

    public HashMap<String, Method> imageCommands = new HashMap<>();

    public TrackingPathFinder(LinearOpMode teleOp) {
        this.teleOp = teleOp;

        Method[] methods = this.teleOp.getClass().getDeclaredMethods();

        for(Method method : methods) {
            if (method.isAnnotationPresent(ImageCommand.class)) {
                String label = Objects.requireNonNull(method.getAnnotation(ImageCommand.class))
                        .name();

                imageCommands.put(label, method);
            }
        }
    }

    public void invoke(String label) throws InvocationTargetException, IllegalAccessException {
        Method method = imageCommands.get(label);

        if (method != null) {
            method.invoke(this.teleOp);
        }
    }
}
