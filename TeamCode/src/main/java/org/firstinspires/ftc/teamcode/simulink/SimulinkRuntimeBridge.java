package org.firstinspires.ftc.teamcode.simulink;

import java.util.Arrays;

/** KEEP AS IS for normal student work. Copies arrays between the FTC and native threads. */
public final class SimulinkRuntimeBridge {
    private static boolean active;
    private static boolean hasOutput;
    private static float[] inputs = new float[0];
    private static float[] outputs = new float[0];

    private SimulinkRuntimeBridge() { }

    public static synchronized void configure(int inputSize, int outputSize) {
        active = false;
        hasOutput = false;
        inputs = new float[inputSize];
        outputs = new float[outputSize];
    }

    public static synchronized void activate() {
        Arrays.fill(inputs, 0.0f);
        Arrays.fill(outputs, 0.0f);
        hasOutput = false;
        active = true;
    }

    public static synchronized void updateInputs(float[] modelInputs) {
        if (active) {
            inputs = modelInputs.clone();
        }
    }

    public static synchronized float[] getInputsForModel() {
        return inputs.clone();
    }

    public static synchronized void publishOutputs(float[] modelOutputs) {
        if (active) {
            outputs = modelOutputs.clone();
            hasOutput = true;
        }
    }

    /** Null until the first output after START. Later reads have no age limit. */
    public static synchronized float[] getLatestOutputs() {
        return active && hasOutput ? outputs.clone() : null;
    }

    public static synchronized void deactivate() {
        active = false;
        hasOutput = false;
        Arrays.fill(inputs, 0.0f);
        Arrays.fill(outputs, 0.0f);
    }
}
