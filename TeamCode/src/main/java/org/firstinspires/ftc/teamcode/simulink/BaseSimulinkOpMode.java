package org.firstinspires.ftc.teamcode.simulink;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.simulink.FtcSimulinkRuntime.FtcSimulinkRuntime;

import java.util.Arrays;

/** KEEP AS IS for normal student work. FTC lifecycle, model exchange, and standard hardware setup. */
public abstract class BaseSimulinkOpMode extends OpMode {
    protected final StandardRobotIo robot = new StandardRobotIo();
    private final float[] inputs;
    private final int outputSize;
    private final DcMotorSimple.Direction[] motorDirections;
    private final RevHubOrientationOnRobot.LogoFacingDirection hubLogo;
    private final RevHubOrientationOnRobot.UsbFacingDirection hubUsb;
    private final int limelightPipeline;

    protected BaseSimulinkOpMode(int inputSize, int outputSize,
                               DcMotorSimple.Direction[] motorDirections,
                               RevHubOrientationOnRobot.LogoFacingDirection hubLogo,
                               RevHubOrientationOnRobot.UsbFacingDirection hubUsb,
                               int limelightPipeline) {
        inputs = new float[inputSize];
        this.outputSize = outputSize;
        this.motorDirections = motorDirections.clone();
        this.hubLogo = hubLogo;
        this.hubUsb = hubUsb;
        this.limelightPipeline = limelightPipeline;
    }

    @Override
    public final void init() {
        SimulinkRuntimeBridge.configure(inputs.length, outputSize);
        // Configure standard devices during INIT, without motor power or servo position commands.
        robot.initialize(hardwareMap, motorDirections, hubLogo, hubUsb, limelightPipeline);
        startModelRuntime();
        telemetry.addData("Frame size", "%d inputs / %d outputs", inputs.length, outputSize);
        telemetry.update();
    }

    @Override
    public final void init_loop() {
        Arrays.fill(inputs, 0.0f);
        readInputs(inputs); // Show measurements before START; model inputs remain zero.
        telemetry.update();
    }

    @Override
    public final void start() {
        SimulinkRuntimeBridge.activate();
    }

    @Override
    public final void loop() {
        Arrays.fill(inputs, 0.0f);
        readInputs(inputs);
        SimulinkRuntimeBridge.updateInputs(inputs);
        float[] outputs = SimulinkRuntimeBridge.getLatestOutputs();
        if (outputs != null) {
            writeOutputs(outputs);
        }
        telemetry.update();
    }

    @Override
    public final void stop() {
        SimulinkRuntimeBridge.deactivate();
        robot.stop(); // Zero motor power and stop camera polling. Do not move servos.
    }

    /** Hardware-free tests can override native startup. */
    protected void startModelRuntime() {
        FtcSimulinkRuntime.startOnce();
    }

    protected abstract void readInputs(float[] inputs);
    protected abstract void writeOutputs(float[] outputs);
}
