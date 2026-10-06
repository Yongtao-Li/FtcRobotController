package org.firstinspires.ftc.teamcode.simulink.io;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/** Maps FTC Robot Configuration names directly to the hardware-type frame. */
public final class FtcHardwareIoAdapter {
    private final HardwareMap hardwareMap;
    private final DcMotorEx[] motors = new DcMotorEx[8];
    private final Servo[] servos = new Servo[8];

    public FtcHardwareIoAdapter(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("Hardware map is required");
        }
        this.hardwareMap = hardwareMap;
    }

    public HardwareInventory initialize(FtcHardwareProfile profile) {
        for (int index = 0; index < motors.length; index++) {
            motors[index] = optionalMotor(index + 1);
        }
        for (int index = 0; index < servos.length; index++) {
            servos[index] = optionalServo(index + 1);
        }
        Set<String> configuredNames = configuredNames();
        List<HardwareInventory.GroupStatus> groups = new ArrayList<>();
        for (FtcHardwareProfile.DeviceGroup group : profile.deviceGroups()) {
            int installedCount = 0;
            for (String deviceName : group.deviceNames()) {
                if (configuredNames.contains(deviceName)) {
                    installedCount++;
                }
            }
            groups.add(new HardwareInventory.GroupStatus(
                    group.displayName(), installedCount, group.deviceNames().size()));
        }
        return new HardwareInventory(groups);
    }

    public void configureMotor(
            int motorIndex,
            DcMotorSimple.Direction direction,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotorEx motor = optionalMotorAt(motorIndex);
        if (motor == null) {
            return;
        }
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void applyCommands(FtcIoFrame commands) {
        for (int index = 0; index < motors.length; index++) {
            if (motors[index] != null) {
                motors[index].setPower(commands.motorPower(index));
            }
        }
        for (int index = 0; index < servos.length; index++) {
            if (servos[index] != null) {
                servos[index].setPosition(commands.servoPosition(index));
            }
        }
    }

    public void populateFeedback(FtcIoFrame feedback) {
        for (int index = 0; index < motors.length; index++) {
            if (motors[index] != null) {
                feedback.setEncoderPositionTicks(index, motors[index].getCurrentPosition());
                feedback.setVelocityTicksPerSec(index, (float) motors[index].getVelocity());
            }
        }
    }

    private DcMotorEx optionalMotor(int number) {
        try {
            return hardwareMap.get(DcMotorEx.class, motorName(number));
        } catch (RuntimeException ignored) {
            return null;
        }
    }

    private Servo optionalServo(int number) {
        try {
            return hardwareMap.get(Servo.class, "Servo" + number);
        } catch (RuntimeException ignored) {
            return null;
        }
    }

    private DcMotorEx optionalMotorAt(int index) {
        if (index < 0 || index >= motors.length) {
            return null;
        }
        return motors[index];
    }

    private Set<String> configuredNames() {
        Set<String> names = new HashSet<>();
        for (HardwareDevice device : hardwareMap.getAll(HardwareDevice.class)) {
            names.addAll(hardwareMap.getNamesOf(device));
        }
        return names;
    }

    public static String motorName(int number) {
        return "DcMotor" + number;
    }

    public static final class HardwareInventory {
        private final List<GroupStatus> groups;

        private HardwareInventory(List<GroupStatus> groups) {
            this.groups = new ArrayList<>(groups);
        }

        public List<GroupStatus> groups() {
            return new ArrayList<>(groups);
        }

        public static final class GroupStatus {
            private final String displayName;
            private final int installedCount;
            private final int supportedCount;

            private GroupStatus(String displayName, int installedCount, int supportedCount) {
                this.displayName = displayName;
                this.installedCount = installedCount;
                this.supportedCount = supportedCount;
            }

            public String displayName() {
                return displayName;
            }

            public int installedCount() {
                return installedCount;
            }

            public int supportedCount() {
                return supportedCount;
            }
        }
    }
}
