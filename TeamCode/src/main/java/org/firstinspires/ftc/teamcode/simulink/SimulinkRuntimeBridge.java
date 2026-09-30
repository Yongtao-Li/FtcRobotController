package org.firstinspires.ftc.teamcode.simulink;

import java.util.Arrays;

/**
 * Thread-safe, fail-closed exchange between FTC OpModes and the generated Simulink runtime.
 */
public final class SimulinkRuntimeBridge {
    public static final int MAX_CHANNEL_COUNT = 256;
    public static final int MIN_CHANNEL_COUNT = 32;
    public static final long OUTPUT_STALE_MILLIS = 250L;

    public static final int INPUT_MODE = 0;
    public static final int INPUT_GAMEPAD1_LEFT_STICK_Y = 1;
    public static final int INPUT_GAMEPAD1_RIGHT_STICK_X = 2;
    public static final int INPUT_GAMEPAD1_LEFT_STICK_X = 3;
    public static final int INPUT_GAMEPAD1_RIGHT_STICK_Y = 4;
    public static final int INPUT_GAMEPAD1_LEFT_TRIGGER = 5;
    public static final int INPUT_GAMEPAD1_RIGHT_TRIGGER = 6;
    public static final int INPUT_GAMEPAD1_A = 7;
    public static final int INPUT_GAMEPAD1_B = 8;
    public static final int INPUT_GAMEPAD1_X = 9;
    public static final int INPUT_GAMEPAD1_Y = 10;
    public static final int INPUT_GAMEPAD1_DPAD_UP = 11;
    public static final int INPUT_GAMEPAD1_DPAD_DOWN = 12;
    public static final int INPUT_GAMEPAD1_DPAD_LEFT = 13;
    public static final int INPUT_GAMEPAD1_DPAD_RIGHT = 14;
    public static final int INPUT_GAMEPAD1_LEFT_BUMPER = 15;
    public static final int INPUT_GAMEPAD1_RIGHT_BUMPER = 16;
    public static final int INPUT_GAMEPAD1_LEFT_STICK_BUTTON = 17;
    public static final int INPUT_GAMEPAD1_RIGHT_STICK_BUTTON = 18;
    public static final int INPUT_GAMEPAD1_START = 19;
    public static final int INPUT_GAMEPAD1_BACK = 20;
    public static final int INPUT_GAMEPAD1_GUIDE = 21;

    public static final int OUTPUT_FRONT_LEFT = 0;
    public static final int OUTPUT_FRONT_RIGHT = 1;
    public static final int OUTPUT_BACK_LEFT = 2;
    public static final int OUTPUT_BACK_RIGHT = 3;

    private static final float[] ZERO_VALUES = new float[MAX_CHANNEL_COUNT];

    private static boolean active;
    private static boolean outputValid;
    private static boolean nativeStarted;
    private static boolean nativeTerminated;
    private static boolean nativeFailed;
    private static boolean nativeLauncherReturned;
    private static int nativeLauncherExitCode;
    private static int outputCount;
    private static long callbackCount;
    private static long firstCallbackNanos;
    private static long lastCallbackNanos;
    private static float[] inputs = ZERO_VALUES.clone();
    private static float[] outputs = ZERO_VALUES.clone();
    private static String nativeMessage = "";
    private static String validationMessage = "";

    private SimulinkRuntimeBridge() {
    }

    public static synchronized void activate(int mode) {
        active = true;
        clearOutputState();
        updateInputsInternal(mode, new float[0]);
    }

    public static synchronized void updateInputs(int mode, float[] payload) {
        if (!active) {
            inputs = ZERO_VALUES.clone();
            return;
        }
        updateInputsInternal(mode, payload);
    }

    public static synchronized float[] getInputsForModel() {
        return active ? inputs.clone() : ZERO_VALUES.clone();
    }

    public static synchronized void publishOutputs(float[] modelOutputs) {
        long now = System.nanoTime();
        callbackCount++;
        if (firstCallbackNanos == 0L) {
            firstCallbackNanos = now;
        }
        lastCallbackNanos = now;

        if (!active) {
            clearOutputState();
            return;
        }
        if (modelOutputs == null
                || modelOutputs.length < MIN_CHANNEL_COUNT
                || modelOutputs.length > MAX_CHANNEL_COUNT) {
            invalidateOutputs("Invalid Simulink output length");
            return;
        }

        for (float value : modelOutputs) {
            if (!Float.isFinite(value)) {
                invalidateOutputs("Non-finite Simulink model output");
                return;
            }
        }
        Arrays.fill(outputs, 0.0f);
        System.arraycopy(modelOutputs, 0, outputs, 0, modelOutputs.length);
        outputCount = modelOutputs.length;
        outputValid = true;
        validationMessage = "";
    }

    public static synchronized Snapshot snapshot() {
        long ageMillis = callbackAgeMillis();
        boolean fresh = lastCallbackNanos != 0L && ageMillis <= OUTPUT_STALE_MILLIS;
        boolean valid = active
                && outputValid
                && fresh
                && !nativeTerminated
                && !nativeFailed;

        String status;
        if (!active) {
            status = "Runtime bridge inactive";
        } else if (nativeFailed) {
            status = nativeMessage.isEmpty() ? "Native runtime failure" : nativeMessage;
        } else if (nativeTerminated) {
            status = "Native runtime terminated";
        } else if (!validationMessage.isEmpty()) {
            status = validationMessage;
        } else if (!fresh) {
            status = "Waiting for a fresh Simulink output";
        } else if (outputValid) {
            status = "Simulink output valid";
        } else {
            status = "Waiting for Simulink output";
        }

        return new Snapshot(
                valid,
                nativeFailed || nativeTerminated,
                outputs.clone(),
                outputCount,
                callbackCount,
                ageMillis,
                callbackFrequencyHz(),
                status);
    }

    public static synchronized void deactivate() {
        active = false;
        inputs = ZERO_VALUES.clone();
        clearOutputState();
        validationMessage = "";
    }

    public static synchronized void recordNativeStarted() {
        nativeStarted = true;
        nativeFailed = false;
        nativeTerminated = false;
        nativeMessage = "Native runtime launcher started";
    }

    public static synchronized void recordNativeLauncherReturn(int exitCode) {
        nativeLauncherReturned = true;
        nativeLauncherExitCode = exitCode;
        nativeMessage = "Native launcher returned " + exitCode;
    }

    public static synchronized void recordNativeMessage(String message) {
        nativeMessage = message == null || message.trim().isEmpty()
                ? "Simulink runtime message"
                : message.trim();
    }

    public static synchronized void recordNativeTermination() {
        nativeTerminated = true;
        active = false;
        inputs = ZERO_VALUES.clone();
        clearOutputState();
        nativeMessage = "Simulink runtime requested termination";
    }

    public static synchronized void recordNativeFailure(String context, Throwable throwable) {
        nativeFailed = true;
        active = false;
        inputs = ZERO_VALUES.clone();
        clearOutputState();
        String detail = throwable == null ? "" : throwable.getClass().getSimpleName();
        String message = throwable == null ? "" : throwable.getMessage();
        nativeMessage = context
                + (detail.isEmpty() ? "" : ": " + detail)
                + (message == null || message.trim().isEmpty() ? "" : " - " + message.trim());
    }

    public static synchronized boolean hasTerminalNativeFault() {
        return nativeFailed || nativeTerminated;
    }

    public static synchronized String nativeStatus() {
        if (nativeFailed || nativeTerminated || !nativeMessage.isEmpty()) {
            return nativeMessage;
        }
        if (!nativeStarted) {
            return "Native runtime not started";
        }
        if (nativeLauncherReturned) {
            return "Native launcher returned " + nativeLauncherExitCode;
        }
        return "Native runtime started";
    }

    private static void updateInputsInternal(int mode, float[] payload) {
        if (mode < 0 || mode > 31 || payload == null
                || payload.length > MAX_CHANNEL_COUNT - 1) {
            inputs = ZERO_VALUES.clone();
            invalidateOutputs("Invalid Simulink input vector");
            return;
        }

        float[] nextInputs = ZERO_VALUES.clone();
        nextInputs[INPUT_MODE] = mode;
        for (int index = 0; index < payload.length; index++) {
            if (!Float.isFinite(payload[index])) {
                inputs = ZERO_VALUES.clone();
                invalidateOutputs("Non-finite Simulink model input");
                return;
            }
            nextInputs[index + 1] = payload[index];
        }
        inputs = nextInputs;
        validationMessage = "";
    }

    private static void invalidateOutputs(String message) {
        clearOutputState();
        validationMessage = message;
    }

    private static void clearOutputState() {
        outputs = ZERO_VALUES.clone();
        outputCount = 0;
        outputValid = false;
    }

    private static long callbackAgeMillis() {
        if (lastCallbackNanos == 0L) {
            return Long.MAX_VALUE;
        }
        return Math.max(0L, (System.nanoTime() - lastCallbackNanos) / 1_000_000L);
    }

    private static double callbackFrequencyHz() {
        if (callbackCount < 2L || firstCallbackNanos == 0L
                || lastCallbackNanos <= firstCallbackNanos) {
            return 0.0;
        }
        return (callbackCount - 1L) * 1_000_000_000.0
                / (lastCallbackNanos - firstCallbackNanos);
    }

    public static final class Snapshot {
        private final boolean valid;
        private final boolean terminalFault;
        private final float[] values;
        private final int outputCount;
        private final long callbackCount;
        private final long ageMillis;
        private final double callbackFrequencyHz;
        private final String status;

        private Snapshot(
                boolean valid,
                boolean terminalFault,
                float[] values,
                int outputCount,
                long callbackCount,
                long ageMillis,
                double callbackFrequencyHz,
                String status) {
            this.valid = valid;
            this.terminalFault = terminalFault;
            this.values = values;
            this.outputCount = outputCount;
            this.callbackCount = callbackCount;
            this.ageMillis = ageMillis;
            this.callbackFrequencyHz = callbackFrequencyHz;
            this.status = status;
        }

        public boolean isValid() {
            return valid;
        }

        public boolean isTerminalFault() {
            return terminalFault;
        }

        public float value(int channel) {
            return channel >= 0 && channel < values.length ? values[channel] : 0.0f;
        }

        public float[] values() {
            return values.clone();
        }

        public int outputCount() {
            return outputCount;
        }

        public long callbackCount() {
            return callbackCount;
        }

        public long ageMillis() {
            return ageMillis;
        }

        public double callbackFrequencyHz() {
            return callbackFrequencyHz;
        }

        public String status() {
            return status;
        }
    }
}
