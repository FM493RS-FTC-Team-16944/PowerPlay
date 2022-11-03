package org.firstinspires.ftc.teamcode.gamepad.easypad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotMovement;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

public class IGamePad {
    private final Gamepad gamepad;
    private final HashMap<String, Object> previousState;

    protected final RobotHardware hardware;
    protected final RobotMovement movement;

    public IGamePad(Robot robot, Gamepad hardwareGamepad) {
        this.hardware = robot.hardware;
        this.movement = robot.movement;

        this.gamepad = hardwareGamepad;
        this.previousState = new HashMap<>();
    }

    public void update() throws NoSuchFieldException, InvocationTargetException, IllegalAccessException {
        List<Method> annotatedMethods = this.getAnnotatedMethods();

        for (Method method : annotatedMethods) {
            Mapping gamePadMap = this.getMap(method);

            if(gamePadMap.map == String.class) {
                invokeWithParameter(method, (String) gamePadMap.map);
            } else if (gamePadMap.map == List.class) {
                invokeWithParameter(method, (List<String>) gamePadMap.map);
            }
        }
    }

    public void invokeWithParameter(Method method, String mapField) throws InvocationTargetException, IllegalAccessException, NoSuchFieldException {
        Object field = this.gamepad.getClass().getField(mapField);

        if (!checkParameterMatch(method, field)) {
            throw new InvalidParameterException(String.format("Invalid parameter type, required %s", field.toString()));
        }

        Object val = this.getOrDefaultPreviousState(mapField, field);

        if (field != val) {
            method.invoke(field);
        }

        this.previousState.put(mapField, field);

        method.setAccessible(false);
    }

    public void invokeWithParameter(Method method, List<String> mapFields) throws InvocationTargetException, IllegalAccessException, NoSuchFieldException {
        if (!checkParameterMatch(method, mapFields)) {
            throw new InvalidParameterException("Invalid parameter type");
        }

        if (comparePrevState(mapFields)) {
            ArrayList<Object> fields = new ArrayList<>();

            for (String mapField : mapFields) {
                fields.add(this.getClass().getField(mapField));
            }

            method.invoke(null, fields);
        }

        for (String mapField : mapFields) {
            Object field = this.gamepad.getClass().getField(mapField);

            this.previousState.put(mapField, field);
        }

        method.setAccessible(false);
    }

    private boolean comparePrevState(List<String> mapFields) throws NoSuchFieldException {
        for (String mapField : mapFields) {
            Object field = this.gamepad.getClass().getField(mapField);

            Object val = this.getOrDefaultPreviousState(mapField, field);

            if(field == val) {
                return false;
            }
        }

        return true;
    }

    private boolean checkParameterMatch(Method method, Object obj) {
        Object[] parameters = method.getParameterTypes();

        if (parameters.length != 1) {
            return false;
        }

        return parameters[0] == obj;
    }

    private boolean checkParameterMatch(Method method, List<String> obj) throws NoSuchFieldException {
        Object[] parameters = method.getParameterTypes();

        if (parameters.length != obj.size()) {
            return false;
        }

        for (String mapField : obj) {
            Object field = this.gamepad.getClass().getField(mapField);

            for (Object param : parameters) {
                if (param != field) {
                    return false;
                }
            }
        }

        return true;
    }

    private Object getOrDefaultPreviousState(String key, Object type) {
        Object val = this.previousState.get(key);

        if (val == null) {
            if (type == Boolean.TYPE) {
                this.previousState.put(key, false);

                return false;
            } else if (type == Float.TYPE) {
                this.previousState.put(key, 0.0f);

                return 0.0f;
            }
        }

        return val;
    }

    private List<Method> getAnnotatedMethods() {
        Class<?> clazz = this.getClass();
        List<Method> methods = new ArrayList<>();

        for (Method method : clazz.getDeclaredMethods()) {
            if (method.isAnnotationPresent(GamePadMap.class)) {
                method.setAccessible(true);
                methods.add(method);
            }
        }

        return methods;
    }

    private Mapping getMap(Method field) {
        return Objects.requireNonNull(field.getAnnotation(GamePadMap.class))
                .map();
    }
}
