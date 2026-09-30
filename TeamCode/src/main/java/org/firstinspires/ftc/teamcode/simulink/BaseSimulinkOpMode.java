package org.firstinspires.ftc.teamcode.simulink;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.simulink.FtcSimulinkRuntime.FtcSimulinkRuntime;

/**
 * Shared lifecycle and telemetry for TeamCode OpModes backed by FtcSimulinkRuntime.
 */
public abstract class BaseSimulinkOpMode extends OpMode {
    private static final int MECANUM_MODE = 1;

    private boolean hardwareReady;
    private boolean runtimeStartAccepted;

    @Override
    public final void init() {
        SimulinkRuntimeBridge.deactivate();
        hardwareReady = initializeHardware();
        runtimeStartAccepted = FtcSimulinkRuntime.startOnce();
        publishTelemetry(SimulinkRuntimeBridge.snapshot());
    }

    @Override
    public final void init_loop() {
        stopActuators();
        publishTelemetry(SimulinkRuntimeBridge.snapshot());
    }

    @Override
    public final void start() {
        stopActuators();
        if (hardwareReady && runtimeStartAccepted) {
            SimulinkRuntimeBridge.activate(MECANUM_MODE);
        }
    }

    @Override
    public final void loop() {
        if (!hardwareReady || !runtimeStartAccepted) {
            stopActuators();
            publishTelemetry(SimulinkRuntimeBridge.snapshot());
            return;
        }

        SimulinkRuntimeBridge.updateInputs(MECANUM_MODE, gamepadPayload(gamepad1));
        SimulinkRuntimeBridge.Snapshot snapshot = SimulinkRuntimeBridge.snapshot();
        if (snapshot.isValid()) {
            applyWheelCommands(snapshot);
            onValidSnapshot(snapshot);
        } else {
            stopActuators();
            onInvalidSnapshot(snapshot);
        }
        publishTelemetry(snapshot);
    }

    @Override
    public final void stop() {
        SimulinkRuntimeBridge.deactivate();
        stopActuators();
    }

    protected boolean initializeHardware() {
        return true;
    }

    protected abstract void applyWheelCommands(SimulinkRuntimeBridge.Snapshot snapshot);

    protected abstract void stopActuators();

    protected void onValidSnapshot(SimulinkRuntimeBridge.Snapshot snapshot) {
    }

    protected void onInvalidSnapshot(SimulinkRuntimeBridge.Snapshot snapshot) {
    }

    protected String hardwareStatus() {
        return hardwareReady ? "Ready" : "Not ready";
    }

    private void publishTelemetry(SimulinkRuntimeBridge.Snapshot snapshot) {
        long age = snapshot.ageMillis();
        telemetry.addData("Hardware", hardwareStatus());
        telemetry.addData("Native library",
                FtcSimulinkRuntime.isLibraryLoaded() ? "Loaded" : "LOAD FAILED");
        telemetry.addData("Runtime", snapshot.status());
        telemetry.addData("Native detail", SimulinkRuntimeBridge.nativeStatus());
        telemetry.addData("Callbacks", snapshot.callbackCount());
        telemetry.addData("Callback rate", "%.1f Hz", snapshot.callbackFrequencyHz());
        telemetry.addData("Callback age", age == Long.MAX_VALUE ? "none" : age + " ms");
        telemetry.addData("Model outputs", snapshot.outputCount());
        telemetry.addData("Input leftY/rightX/leftX", "%.2f / %.2f / %.2f",
                gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
        telemetry.addData("Front L / R", "%.2f / %.2f",
                snapshot.value(SimulinkRuntimeBridge.OUTPUT_FRONT_LEFT),
                snapshot.value(SimulinkRuntimeBridge.OUTPUT_FRONT_RIGHT));
        telemetry.addData("Back L / R", "%.2f / %.2f",
                snapshot.value(SimulinkRuntimeBridge.OUTPUT_BACK_LEFT),
                snapshot.value(SimulinkRuntimeBridge.OUTPUT_BACK_RIGHT));
        telemetry.update();
    }

    private static float[] gamepadPayload(Gamepad gamepad) {
        float[] payload = new float[SimulinkRuntimeBridge.INPUT_GAMEPAD1_GUIDE];
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_LEFT_STICK_Y - 1] =
                gamepad.left_stick_y;
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_RIGHT_STICK_X - 1] =
                gamepad.right_stick_x;
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_LEFT_STICK_X - 1] =
                gamepad.left_stick_x;
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_RIGHT_STICK_Y - 1] =
                gamepad.right_stick_y;
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_LEFT_TRIGGER - 1] =
                gamepad.left_trigger;
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_RIGHT_TRIGGER - 1] =
                gamepad.right_trigger;
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_A - 1] = asFloat(gamepad.a);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_B - 1] = asFloat(gamepad.b);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_X - 1] = asFloat(gamepad.x);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_Y - 1] = asFloat(gamepad.y);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_DPAD_UP - 1] =
                asFloat(gamepad.dpad_up);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_DPAD_DOWN - 1] =
                asFloat(gamepad.dpad_down);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_DPAD_LEFT - 1] =
                asFloat(gamepad.dpad_left);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_DPAD_RIGHT - 1] =
                asFloat(gamepad.dpad_right);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_LEFT_BUMPER - 1] =
                asFloat(gamepad.left_bumper);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_RIGHT_BUMPER - 1] =
                asFloat(gamepad.right_bumper);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_LEFT_STICK_BUTTON - 1] =
                asFloat(gamepad.left_stick_button);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_RIGHT_STICK_BUTTON - 1] =
                asFloat(gamepad.right_stick_button);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_START - 1] =
                asFloat(gamepad.start);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_BACK - 1] =
                asFloat(gamepad.back);
        payload[SimulinkRuntimeBridge.INPUT_GAMEPAD1_GUIDE - 1] =
                asFloat(gamepad.guide);
        return payload;
    }

    private static float asFloat(boolean value) {
        return value ? 1.0f : 0.0f;
    }
}
