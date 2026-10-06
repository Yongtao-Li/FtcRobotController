package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.simulink.BaseSimulinkOpMode;
import org.firstinspires.ftc.teamcode.simulink.io.FtcHardwareIoAdapter;

/** Simulink TeleOp using the standard optional FTC hardware profile. */
@TeleOp(name = "Simulink TeleOp", group = "Simulink")
public final class SimulinkTeleOp extends BaseSimulinkOpMode {
    private static final DcMotorSimple.Direction[] MOTOR_DIRECTIONS = {
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD
    };

    @Override
    protected void configureHardware(FtcHardwareIoAdapter adapter) {
        for (int index = 0; index < MOTOR_DIRECTIONS.length; index++) {
            adapter.configureMotor(index, MOTOR_DIRECTIONS[index], DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
