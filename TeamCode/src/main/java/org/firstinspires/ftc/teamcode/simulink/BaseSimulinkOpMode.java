package org.firstinspires.ftc.teamcode.simulink;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.simulink.io.FtcHardwareIoAdapter;
import org.firstinspires.ftc.teamcode.simulink.io.FtcIoFrame;
import org.firstinspires.ftc.teamcode.simulink.io.FtcHardwareProfile;
import org.simulink.FtcSimulinkRuntime.FtcSimulinkRuntime;

/** Shared FTC lifecycle for the 256-channel hardware-type frame. */
public abstract class BaseSimulinkOpMode extends OpMode {
    private static final String[] LIVE_INDICATORS = {"◐", "◓", "◑", "◒"};

    private FtcHardwareIoAdapter hardwareAdapter;
    private FtcHardwareIoAdapter.HardwareInventory hardwareInventory;
    private int telemetryFrame;

    @Override
    public final void init() {
        SimulinkRuntimeBridge.deactivate();
        hardwareAdapter = new FtcHardwareIoAdapter(hardwareMap);
        hardwareInventory = hardwareAdapter.initialize(hardwareProfile());
        configureHardware(hardwareAdapter);
        stopActuators();
        FtcSimulinkRuntime.startOnce();
    }

    @Override
    public final void start() {
        stopActuators();
        SimulinkRuntimeBridge.activate();
    }

    @Override
    public final void loop() {
        float[] inputValues = new float[FtcIoFrame.FRAME_LENGTH];
        setGamepad(inputValues, gamepad1, gamepad2);
        FtcIoFrame inputs = new FtcIoFrame(inputValues);
        hardwareAdapter.populateFeedback(inputs);
        SimulinkRuntimeBridge.updateInputs(inputs.values());
        SimulinkRuntimeBridge.Snapshot snapshot = SimulinkRuntimeBridge.snapshot();
        if (snapshot.isValid()) {
            hardwareAdapter.applyCommands(new FtcIoFrame(snapshot.values()));
        } else {
            stopActuators();
        }
        displayHardwareInventory();
        displayDriverInputs();
        telemetry.addLine("═══════ Runtime ═══════");
        telemetry.addData("Simulink", snapshot.status());
        telemetry.addData("Native", SimulinkRuntimeBridge.nativeStatus());
        telemetry.update();
    }

    @Override
    public final void stop() {
        SimulinkRuntimeBridge.deactivate();
        stopActuators();
    }

    protected FtcHardwareProfile hardwareProfile() {
        return FtcHardwareProfile.STANDARD;
    }

    protected abstract void configureHardware(FtcHardwareIoAdapter adapter);

    private void stopActuators() {
        if (hardwareAdapter != null) {
            hardwareAdapter.applyCommands(new FtcIoFrame());
        }
    }

    private void displayHardwareInventory() {
        telemetry.addLine("═══════ Hardware ═══════");
        for (FtcHardwareIoAdapter.HardwareInventory.GroupStatus group : hardwareInventory.groups()) {
            telemetry.addData(
                    group.displayName(),
                    "%d installed / %d supported",
                    group.installedCount(),
                    group.supportedCount());
        }
    }

    private void displayDriverInputs() {
        String liveIndicator = LIVE_INDICATORS[(telemetryFrame++ / 8) % LIVE_INDICATORS.length];
        telemetry.addLine("🎮 Driver Inputs " + liveIndicator);
        telemetry.addData(
                "Gamepad 1",
                "LS (%.2f, %.2f)  RS (%.2f, %.2f)",
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.right_stick_y);
        telemetry.addData("G1 buttons", pressedButtons(gamepad1));
        telemetry.addData(
                "Gamepad 2",
                "LS (%.2f, %.2f)  RS (%.2f, %.2f)",
                gamepad2.left_stick_x,
                gamepad2.left_stick_y,
                gamepad2.right_stick_x,
                gamepad2.right_stick_y);
        telemetry.addData("G2 buttons", pressedButtons(gamepad2));
    }

    private static String pressedButtons(Gamepad gamepad) {
        StringBuilder buttons = new StringBuilder();
        appendPressed(buttons, gamepad.a, "A");
        appendPressed(buttons, gamepad.b, "B");
        appendPressed(buttons, gamepad.x, "X");
        appendPressed(buttons, gamepad.y, "Y");
        appendPressed(buttons, gamepad.left_bumper, "LB");
        appendPressed(buttons, gamepad.right_bumper, "RB");
        appendPressed(buttons, gamepad.dpad_up, "D↑");
        appendPressed(buttons, gamepad.dpad_down, "D↓");
        appendPressed(buttons, gamepad.dpad_left, "D←");
        appendPressed(buttons, gamepad.dpad_right, "D→");
        return buttons.length() == 0 ? "—" : buttons.toString();
    }

    private static void appendPressed(StringBuilder buttons, boolean pressed, String label) {
        if (!pressed) {
            return;
        }
        if (buttons.length() > 0) {
            buttons.append(' ');
        }
        buttons.append(label);
    }

    private static void setGamepad(float[] frame, Gamepad gamepad1, Gamepad gamepad2) {
        writeGamepad(frame, 0, gamepad1);
        writeGamepad(frame, 21, gamepad2);
    }

    private static void writeGamepad(float[] frame, int offset, Gamepad gamepad) {
        frame[offset] = gamepad.left_stick_x;
        frame[offset + 1] = gamepad.left_stick_y;
        frame[offset + 2] = gamepad.right_stick_x;
        frame[offset + 3] = gamepad.right_stick_y;
        frame[offset + 4] = asFloat(gamepad.dpad_up);
        frame[offset + 5] = asFloat(gamepad.dpad_down);
        frame[offset + 6] = asFloat(gamepad.dpad_left);
        frame[offset + 7] = asFloat(gamepad.dpad_right);
        frame[offset + 8] = asFloat(gamepad.a);
        frame[offset + 9] = asFloat(gamepad.b);
        frame[offset + 10] = asFloat(gamepad.x);
        frame[offset + 11] = asFloat(gamepad.y);
        frame[offset + 12] = asFloat(gamepad.guide);
        frame[offset + 13] = asFloat(gamepad.start);
        frame[offset + 14] = asFloat(gamepad.back);
        frame[offset + 15] = asFloat(gamepad.left_bumper);
        frame[offset + 16] = asFloat(gamepad.right_bumper);
        frame[offset + 17] = asFloat(gamepad.left_stick_button);
        frame[offset + 18] = asFloat(gamepad.right_stick_button);
        frame[offset + 19] = gamepad.left_trigger;
        frame[offset + 20] = gamepad.right_trigger;
    }

    private static float asFloat(boolean value) {
        return value ? 1.0f : 0.0f;
    }
}
