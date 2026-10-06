package org.firstinspires.ftc.teamcode.simulink;

import org.firstinspires.ftc.teamcode.simulink.io.FtcIoFrame;

import java.util.Arrays;

/** Thread-safe, fail-closed exchange between TeamCode and the generated model. */
public final class SimulinkRuntimeBridge {
    public static final int CHANNEL_COUNT = FtcIoFrame.FRAME_LENGTH;
    public static final long OUTPUT_STALE_MILLIS = 250L;

    private static final float[] ZERO_VALUES = new float[CHANNEL_COUNT];
    private static boolean active;
    private static boolean outputValid;
    private static boolean nativeFailed;
    private static boolean nativeTerminated;
    private static long lastCallbackNanos;
    private static float[] inputs = ZERO_VALUES.clone();
    private static float[] outputs = ZERO_VALUES.clone();
    private static String nativeMessage = "";
    private static String validationMessage = "";

    private SimulinkRuntimeBridge() {
    }

    public static synchronized void activate() {
        active = true;
        outputValid = false;
        outputs = ZERO_VALUES.clone();
    }

    public static synchronized void updateInputs(float[] modelInputs) {
        if (!active || !validFrame(modelInputs)) {
            inputs = ZERO_VALUES.clone();
            return;
        }
        inputs = modelInputs.clone();
    }

    public static synchronized float[] getInputsForModel() {
        return active ? inputs.clone() : ZERO_VALUES.clone();
    }

    public static synchronized void publishOutputs(float[] modelOutputs) {
        lastCallbackNanos = System.nanoTime();
        if (!active || !validFrame(modelOutputs)) {
            outputValid = false;
            outputs = ZERO_VALUES.clone();
            validationMessage = "Invalid Simulink output frame";
            return;
        }
        outputs = modelOutputs.clone();
        outputValid = true;
        validationMessage = "";
    }

    public static synchronized Snapshot snapshot() {
        long ageMillis = lastCallbackNanos == 0L ? Long.MAX_VALUE
                : (System.nanoTime() - lastCallbackNanos) / 1_000_000L;
        boolean valid = active && outputValid && ageMillis <= OUTPUT_STALE_MILLIS
                && !nativeFailed && !nativeTerminated;
        String status = valid ? "Simulink output valid"
                : !validationMessage.isEmpty() ? validationMessage
                : nativeFailed ? nativeMessage
                : nativeTerminated ? "Native runtime terminated"
                : "Waiting for a fresh Simulink output";
        return new Snapshot(valid, nativeFailed || nativeTerminated, outputs.clone(), status);
    }

    public static synchronized void deactivate() {
        active = false;
        inputs = ZERO_VALUES.clone();
        outputs = ZERO_VALUES.clone();
        outputValid = false;
    }

    public static synchronized void recordNativeStarted() {
        nativeFailed = false;
        nativeTerminated = false;
        nativeMessage = "Native runtime started";
    }

    public static synchronized void recordNativeLauncherReturn(int exitCode) {
        nativeMessage = "Native launcher returned " + exitCode;
    }

    public static synchronized void recordNativeMessage(String message) {
        nativeMessage = message == null ? "" : message;
    }

    public static synchronized void recordNativeTermination() {
        nativeTerminated = true;
        deactivate();
    }

    public static synchronized void recordNativeFailure(String context, Throwable throwable) {
        nativeFailed = true;
        nativeMessage = context + (throwable == null ? "" : ": " + throwable.getMessage());
        deactivate();
    }

    public static synchronized boolean hasTerminalNativeFault() {
        return nativeFailed || nativeTerminated;
    }

    public static synchronized String nativeStatus() {
        return nativeMessage;
    }

    private static boolean validFrame(float[] frame) {
        if (frame == null || frame.length != CHANNEL_COUNT) {
            return false;
        }
        for (float value : frame) {
            if (!Float.isFinite(value)) {
                return false;
            }
        }
        return true;
    }

    public static final class Snapshot {
        private final boolean valid;
        private final boolean terminalFault;
        private final float[] values;
        private final String status;

        private Snapshot(boolean valid, boolean terminalFault, float[] values, String status) {
            this.valid = valid;
            this.terminalFault = terminalFault;
            this.values = values;
            this.status = status;
        }

        public boolean isValid() {
            return valid;
        }

        public boolean isTerminalFault() {
            return terminalFault;
        }

        public float[] values() {
            return Arrays.copyOf(values, values.length);
        }

        public String status() {
            return status;
        }
    }
}
