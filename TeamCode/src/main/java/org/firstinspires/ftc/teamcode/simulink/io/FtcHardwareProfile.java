package org.firstinspires.ftc.teamcode.simulink.io;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/** Standard FTC device names and maximum quantities for the Simulink hardware frame. */
public final class FtcHardwareProfile {
    public static final FtcHardwareProfile STANDARD = new FtcHardwareProfile(Arrays.asList(
            DeviceGroup.numbered("DC Motors", "DcMotor", 8),
            DeviceGroup.numbered("Servos", "Servo", 8),
            DeviceGroup.named("IMU", "IMU"),
            DeviceGroup.numbered("Color + Distance", "ColorDistanceSensor", 2),
            DeviceGroup.numbered("Touch Switches", "TouchSwitch", 4),
            DeviceGroup.numbered("Analog Position", "AnalogPositionSensor", 2),
            DeviceGroup.named("Limelight", "Limelight")));

    private final List<DeviceGroup> deviceGroups;

    private FtcHardwareProfile(List<DeviceGroup> deviceGroups) {
        this.deviceGroups = Collections.unmodifiableList(deviceGroups);
    }

    public List<DeviceGroup> deviceGroups() {
        return deviceGroups;
    }

    public static final class DeviceGroup {
        private final String displayName;
        private final List<String> deviceNames;

        private DeviceGroup(String displayName, List<String> deviceNames) {
            this.displayName = displayName;
            this.deviceNames = Collections.unmodifiableList(deviceNames);
        }

        public static DeviceGroup numbered(String displayName, String namePrefix, int maximumCount) {
            String[] names = new String[maximumCount];
            for (int index = 0; index < maximumCount; index++) {
                names[index] = namePrefix + (index + 1);
            }
            return new DeviceGroup(displayName, Arrays.asList(names));
        }

        public static DeviceGroup named(String displayName, String deviceName) {
            return new DeviceGroup(displayName, Collections.singletonList(deviceName));
        }

        public String displayName() {
            return displayName;
        }

        public List<String> deviceNames() {
            return deviceNames;
        }
    }
}
