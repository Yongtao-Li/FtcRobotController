package org.firstinspires.ftc.teamcode.simulink.io;

import java.util.Arrays;

/** Fixed 256-element FTC Simulink transport frame. */
public final class FtcIoFrame {
    public static final int FRAME_LENGTH = 256;
    public static final int INPUT_MOTOR_POSITION_START = 42;
    public static final int INPUT_MOTOR_VELOCITY_START = 50;
    public static final int OUTPUT_MOTOR_POWER_START = 0;
    public static final int OUTPUT_SERVO_POSITION_START = 8;

    private final float[] values;

    public FtcIoFrame() {
        values = new float[FRAME_LENGTH];
    }

    public FtcIoFrame(float[] source) {
        if (source == null || source.length != FRAME_LENGTH) {
            throw new IllegalArgumentException("Frame must contain 256 values");
        }
        values = source.clone();
    }

    public float[] values() {
        return values.clone();
    }

    public void clear() {
        Arrays.fill(values, 0.0f);
    }

    public void setEncoderPositionTicks(int motorIndex, int ticks) {
        values[INPUT_MOTOR_POSITION_START + motorIndex] = ticks;
    }

    public void setVelocityTicksPerSec(int motorIndex, float ticksPerSec) {
        values[INPUT_MOTOR_VELOCITY_START + motorIndex] = ticksPerSec;
    }

    public float motorPower(int motorIndex) {
        return clamp(values[OUTPUT_MOTOR_POWER_START + motorIndex], -1.0f, 1.0f);
    }

    public float servoPosition(int servoIndex) {
        return clamp(values[OUTPUT_SERVO_POSITION_START + servoIndex], 0.0f, 1.0f);
    }

    private static float clamp(float value, float lower, float upper) {
        return Math.max(lower, Math.min(upper, value));
    }
}
