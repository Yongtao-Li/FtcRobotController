package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.simulink.BaseSimulinkOpMode;
import org.firstinspires.ftc.teamcode.simulink.SimulinkRuntimeBridge;

/**
 * Four-motor FTC adapter for the wheel commands produced by FtcSimulinkRuntime.
 */
@TeleOp(name = "Simulink Mecanum", group = "Simulink")
public final class SimulinkMecanum extends BaseSimulinkOpMode {
    private static final long INVALID_OUTPUT_STOP_MILLIS = 500L;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private String hardwareMessage = "Not initialized";
    private long invalidSinceNanos;

    @Override
    protected boolean initializeHardware() {
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            stopActuators();
            hardwareMessage = "Four motors ready";
            return true;
        } catch (RuntimeException exception) {
            hardwareMessage = "Missing/invalid drivetrain: " + exception.getMessage();
            stopActuators();
            return false;
        }
    }

    @Override
    protected void applyWheelCommands(SimulinkRuntimeBridge.Snapshot snapshot) {
        invalidSinceNanos = 0L;
        frontLeft.setPower(clamp(snapshot.value(SimulinkRuntimeBridge.OUTPUT_FRONT_LEFT)));
        frontRight.setPower(clamp(snapshot.value(SimulinkRuntimeBridge.OUTPUT_FRONT_RIGHT)));
        backLeft.setPower(clamp(snapshot.value(SimulinkRuntimeBridge.OUTPUT_BACK_LEFT)));
        backRight.setPower(clamp(snapshot.value(SimulinkRuntimeBridge.OUTPUT_BACK_RIGHT)));
    }

    @Override
    protected void stopActuators() {
        setPowerIfPresent(frontLeft, 0.0);
        setPowerIfPresent(frontRight, 0.0);
        setPowerIfPresent(backLeft, 0.0);
        setPowerIfPresent(backRight, 0.0);
    }

    @Override
    protected void onInvalidSnapshot(SimulinkRuntimeBridge.Snapshot snapshot) {
        if (snapshot.isTerminalFault()) {
            requestOpModeStop();
            return;
        }

        if (invalidSinceNanos == 0L) {
            invalidSinceNanos = System.nanoTime();
        }
        long invalidMillis = (System.nanoTime() - invalidSinceNanos) / 1_000_000L;
        if (invalidMillis > INVALID_OUTPUT_STOP_MILLIS) {
            requestOpModeStop();
        }
    }

    @Override
    protected String hardwareStatus() {
        return hardwareMessage;
    }

    private static double clamp(float command) {
        return Math.max(-1.0, Math.min(1.0, command));
    }

    private static void setPowerIfPresent(DcMotor motor, double power) {
        if (motor != null) {
            motor.setPower(power);
        }
    }
}
