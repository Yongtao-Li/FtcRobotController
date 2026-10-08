package org.simulink.FtcSimulinkRuntime;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.simulink.SimulinkRuntimeBridge;

/** KEEP AS IS. Class name, native declarations, and callbacks match the generated library. */
public final class FtcSimulinkRuntime {
    private static final FtcSimulinkRuntime INSTANCE = new FtcSimulinkRuntime();
    private static boolean startAttempted;

    static {
        System.loadLibrary("FtcSimulinkRuntime");
    }

    private FtcSimulinkRuntime() { }

    /** Start the generated scheduler once per Robot Controller app process. */
    public static synchronized void startOnce() {
        if (startAttempted) {
            return;
        }
        startAttempted = true;
        Thread launcher = new Thread(() -> INSTANCE.naMain(
                new String[]{"TeamCode", "FtcSimulinkRuntime"}, INSTANCE),
                "FtcSimulinkRuntime-Launcher");
        launcher.setDaemon(true);
        launcher.start();
    }

    public float[] getModelInputs() {
        return SimulinkRuntimeBridge.getInputsForModel();
    }

    public void setModelOutputs(float[] modelOutputs) {
        SimulinkRuntimeBridge.publishOutputs(modelOutputs);
    }

    public void flashMessage(String message) {
        RobotLog.ii("FtcSimulinkRuntime", "%s", message);
    }

    /** The native callback must not close the FTC Robot Controller activity. */
    public void terminateApp() {
        RobotLog.ii("FtcSimulinkRuntime", "Native runtime terminated");
    }

    private native int naMain(String[] argv, FtcSimulinkRuntime runtime);

    @SuppressWarnings("unused")
    private native void naOnAppStateChange(int state);
}
