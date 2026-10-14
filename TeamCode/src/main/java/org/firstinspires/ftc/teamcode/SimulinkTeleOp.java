package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.simulink.BaseSimulinkOpMode;

/** Student settings and custom I/O. Java array indices start at zero. */
@Config("FlywheelPID")
@TeleOp(name = "Simulink TeleOp", group = "Simulink")
public final class SimulinkTeleOp extends BaseSimulinkOpMode {
    // EDIT HERE: array sizes. Match the model and rebuild both libraries after a change.
    public static final int INPUT_SIZE = 256;
    public static final int OUTPUT_SIZE = 256;

    // EDIT HERE: robot settings. Standard hardware names are in the workbook.
    private static final DcMotorSimple.Direction[] MOTOR_DIRECTIONS = {
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD
    };
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private static final int LIMELIGHT_PIPELINE = 0;

    // EDIT HERE: PID defaults. These public static fields are tunable in Dashboard.
    // Open http://192.168.43.1:8080/dash on the Control Hub network.
    // In Config, expand FlywheelPID. Edit target RPM or gains and save to apply live.
    // volatile makes Dashboard changes visible to the FTC loop. No restart is needed.
    // Dashboard edits are temporary. Change these defaults to keep values after an app restart.
    // Simulink uses requested target RPM directly and owns enable, PID, and power limits.
    // A nonpositive target disables the controller. Java does not check gains.
    // Demo assumptions: direct drive, 28 encoder counts/revolution, 6000 RPM at full power.
    // Kv uses normalized motor power per RPM, not volts per RPM. Tune on the real motor.
    public static volatile double FLYWHEEL_KV = 1.0 / 6000.0;
    public static volatile double FLYWHEEL_KP = 0.0001;
    public static volatile double FLYWHEEL_KI = 0.0;
    public static volatile double FLYWHEEL_KD = 0.0;
    public static volatile double FLYWHEEL_TARGET_RPM = 3000.0;
    private static final int IN_FLYWHEEL_KP = 86; // Simulink Selector index 87
    private static final int IN_FLYWHEEL_KI = 87; // Simulink Selector index 88
    private static final int IN_FLYWHEEL_KD = 88; // Simulink Selector index 89
    private static final int IN_FLYWHEEL_KV = 89; // Simulink Selector index 90
    private static final int IN_FLYWHEEL_TARGET_RPM = 90; // Simulink Selector index 91
    private static final int OUT_FLYWHEEL_MEASURED_RPM = 16;
    private static final int OUT_FLYWHEEL_FEEDFORWARD = 17;
    private static final int OUT_FLYWHEEL_FEEDBACK = 18;
    private static final int OUT_FLYWHEEL_ENABLED = 19;
    private float requestedTargetRpm;

    // KEEP AS IS: pass these settings to the parent class. It manages INIT and STOP.
    public SimulinkTeleOp() {
        super(INPUT_SIZE, OUTPUT_SIZE, MOTOR_DIRECTIONS, HUB_LOGO, HUB_USB, LIMELIGHT_PIPELINE);
    }

    // EDIT HERE: declare additional hardware, if needed.

    @Override
    protected void readInputs(float[] inputs) {
        // KEEP AS IS: standard feedback is at inputs 0..85. Runtime clears unused slots.
        robot.readInputs(inputs, gamepad1, gamepad2);
        // Read the current Dashboard gains each cycle and send them to Simulink as single values.
        inputs[IN_FLYWHEEL_KP] = (float) FLYWHEEL_KP;
        inputs[IN_FLYWHEEL_KI] = (float) FLYWHEEL_KI;
        inputs[IN_FLYWHEEL_KD] = (float) FLYWHEEL_KD;
        inputs[IN_FLYWHEEL_KV] = (float) FLYWHEEL_KV;
        requestedTargetRpm = (float) FLYWHEEL_TARGET_RPM;
        inputs[IN_FLYWHEEL_TARGET_RPM] = requestedTargetRpm;

        // EDIT HERE: custom inputs. Inputs 86..90 carry Kp, Ki, Kd, Kv, target RPM.
        // Inputs 91..255 are free.
        telemetry.addData("Flywheel speed (ticks/s)", inputs[54]);
    }

    @Override
    protected void writeOutputs(float[] outputs) {
        // KEEP AS IS: outputs 0..7 -> motor power; 8..15 -> servo command.
        // FTC configuration selects position [0,1] or continuous-rotation power [-1,1].
        // Zero moves a positional servo to an endpoint; zero stops a CR servo.
        robot.writeOutputs(outputs);

        // Graph RPM together; graph normalized power terms separately in Dashboard.
        // Plot the requested target sent to the model. Measurements are the latest
        // asynchronous model output frame and may lag a target edit by a cycle.
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("flywheel/targetRPM", requestedTargetRpm);
        packet.put("flywheel/measuredRPM", outputs[OUT_FLYWHEEL_MEASURED_RPM]);
        packet.put("flywheel/feedforwardPower", outputs[OUT_FLYWHEEL_FEEDFORWARD]);
        packet.put("flywheel/feedbackPower", outputs[OUT_FLYWHEEL_FEEDBACK]);
        packet.put("flywheel/motorPower", outputs[4]);
        packet.put("flywheel/enabled", outputs[OUT_FLYWHEEL_ENABLED]);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        // EDIT HERE: custom outputs. Outputs 20..255 are free.
    }
}
