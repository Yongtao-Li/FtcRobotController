package org.firstinspires.ftc.teamcode.simulink;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/** KEEP AS IS for normal student work. Standard hardware names and fixed signal positions. */
public final class StandardRobotIo {
    private final DcMotorEx[] motors = new DcMotorEx[8];
    private final Servo[] servos = new Servo[8];
    private final DistanceSensor[] distanceSensors = new DistanceSensor[2];
    private final ColorSensor[] colorSensors = new ColorSensor[2];
    private final TouchSensor[] touchSensors = new TouchSensor[4];
    private final AnalogInput[] analogSensors = new AnalogInput[2];
    private IMU imu;
    private Limelight3A limelight;

    public void initialize(HardwareMap hardwareMap, DcMotorSimple.Direction[] directions,
                           RevHubOrientationOnRobot.LogoFacingDirection logo,
                           RevHubOrientationOnRobot.UsbFacingDirection usb, int pipeline) {
        for (int i = 0; i < motors.length; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, "DcMotor" + (i + 1));
            if (motors[i] != null) {
                motors[i].setDirection(directions[i]);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            servos[i] = hardwareMap.tryGet(Servo.class, "Servo" + (i + 1));
        }
        imu = hardwareMap.tryGet(IMU.class, "IMU");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        }
        for (int i = 0; i < distanceSensors.length; i++) {
            String name = "ColorDistanceSensor" + (i + 1);
            distanceSensors[i] = hardwareMap.tryGet(DistanceSensor.class, name);
            colorSensors[i] = hardwareMap.tryGet(ColorSensor.class, name);
        }
        for (int i = 0; i < touchSensors.length; i++) {
            touchSensors[i] = hardwareMap.tryGet(TouchSensor.class, "TouchSwitch" + (i + 1));
        }
        for (int i = 0; i < analogSensors.length; i++) {
            analogSensors[i] = hardwareMap.tryGet(AnalogInput.class, "AnalogPositionSensor" + (i + 1));
        }
        limelight = hardwareMap.tryGet(Limelight3A.class, "Limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
            limelight.start();
        }
    }

    public void readInputs(float[] inputs, Gamepad gamepad1, Gamepad gamepad2) {
        writeGamepad(inputs, 0, gamepad1);
        writeGamepad(inputs, 21, gamepad2);
        for (int i = 0; i < motors.length; i++) {
            if (motors[i] != null) {
                inputs[42 + i] = motors[i].getCurrentPosition(); // encoder ticks
                inputs[50 + i] = (float) motors[i].getVelocity(); // ticks/second
            }
        }
        if (imu != null) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            AngularVelocity rate = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            inputs[58] = (float) angles.getYaw(AngleUnit.RADIANS);
            inputs[59] = (float) angles.getPitch(AngleUnit.RADIANS);
            inputs[60] = (float) angles.getRoll(AngleUnit.RADIANS);
            inputs[61] = rate.xRotationRate;
            inputs[62] = rate.yRotationRate;
            inputs[63] = rate.zRotationRate;
        }
        for (int i = 0; i < distanceSensors.length; i++) {
            int start = 64 + i * 5; // meters, raw R, G, B, A
            if (distanceSensors[i] != null) {
                inputs[start] = (float) distanceSensors[i].getDistance(DistanceUnit.METER);
            }
            if (colorSensors[i] != null) {
                inputs[start + 1] = colorSensors[i].red();
                inputs[start + 2] = colorSensors[i].green();
                inputs[start + 3] = colorSensors[i].blue();
                inputs[start + 4] = colorSensors[i].alpha();
            }
        }
        for (int i = 0; i < touchSensors.length; i++) {
            if (touchSensors[i] != null) {
                inputs[74 + i] = touchSensors[i].isPressed() ? 1 : 0;
            }
        }
        for (int i = 0; i < analogSensors.length; i++) {
            if (analogSensors[i] != null) {
                inputs[78 + i] = (float) analogSensors[i].getVoltage();
            }
        }
        if (limelight != null) {
            inputs[80] = limelight.isConnected() ? 1 : 0;
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                inputs[81] = result.isValid() ? 1 : 0;
                inputs[82] = (float) result.getTx(); // degrees
                inputs[83] = (float) result.getTy(); // degrees
                inputs[84] = (float) result.getTa(); // percent
                inputs[85] = (float) (result.getCaptureLatency()
                        + result.getTargetingLatency() + result.getParseLatency()); // ms
            }
        }
    }

    public void writeOutputs(float[] outputs) {
        for (int i = 0; i < motors.length; i++) {
            if (motors[i] != null) {
                motors[i].setPower(outputs[i]);
            }
            if (servos[i] != null) {
                servos[i].setPosition(outputs[8 + i]);
            }
        }
    }

    public void stop() {
        for (DcMotorEx motor : motors) {
            if (motor != null) {
                motor.setPower(0);
            }
        }
        if (limelight != null) {
            limelight.stop();
        }
    }

    private static void writeGamepad(float[] inputs, int start, Gamepad gamepad) {
        inputs[start] = gamepad.left_stick_x;
        inputs[start + 1] = gamepad.left_stick_y;
        inputs[start + 2] = gamepad.right_stick_x;
        inputs[start + 3] = gamepad.right_stick_y;
        inputs[start + 4] = gamepad.dpad_up ? 1 : 0;
        inputs[start + 5] = gamepad.dpad_down ? 1 : 0;
        inputs[start + 6] = gamepad.dpad_left ? 1 : 0;
        inputs[start + 7] = gamepad.dpad_right ? 1 : 0;
        inputs[start + 8] = gamepad.a ? 1 : 0;
        inputs[start + 9] = gamepad.b ? 1 : 0;
        inputs[start + 10] = gamepad.x ? 1 : 0;
        inputs[start + 11] = gamepad.y ? 1 : 0;
        inputs[start + 12] = gamepad.guide ? 1 : 0;
        inputs[start + 13] = gamepad.start ? 1 : 0;
        inputs[start + 14] = gamepad.back ? 1 : 0;
        inputs[start + 15] = gamepad.left_bumper ? 1 : 0;
        inputs[start + 16] = gamepad.right_bumper ? 1 : 0;
        inputs[start + 17] = gamepad.left_stick_button ? 1 : 0;
        inputs[start + 18] = gamepad.right_stick_button ? 1 : 0;
        inputs[start + 19] = gamepad.left_trigger;
        inputs[start + 20] = gamepad.right_trigger;
    }
}
