package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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
    // In Config, expand FlywheelPID. Edit the gains and save to apply them live.
    // volatile makes Dashboard changes visible to the FTC loop. No restart is needed.
    // Dashboard edits are temporary. Change these defaults to keep values after an app restart.
    // Simulink owns target speed, enable, PID, and output limits. Java does not check gains.
    public static volatile double FLYWHEEL_KP = 0.0;
    public static volatile double FLYWHEEL_KI = 0.0;
    public static volatile double FLYWHEEL_KD = 0.0;
    private static final int IN_FLYWHEEL_KP = 86; // Simulink Selector index 87
    private static final int IN_FLYWHEEL_KI = 87; // Simulink Selector index 88
    private static final int IN_FLYWHEEL_KD = 88; // Simulink Selector index 89

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

        // EDIT HERE: custom inputs. Inputs 86..88 carry the three PID gains.
        // Inputs 89..255 are free. Example: elapsed seconds at Selector index 90.
        // inputs[89] = (float) getRuntime();
        telemetry.addData("Flywheel speed (ticks/s)", inputs[54]);
    }

    @Override
    protected void writeOutputs(float[] outputs) {
        // KEEP AS IS: outputs 0..7 -> motor power; 8..15 -> servo position.
        // Values go directly to the SDK. A servo value of zero commands position zero.
        robot.writeOutputs(outputs);

        // EDIT HERE: custom outputs. Outputs 16..255 are free.
        // telemetry.addData("Model debug", outputs[16]); // Simulink element 17
    }
}
