package com.spikes2212.control;

import java.util.function.Supplier;

/**
 * The constants used in a FeedForward calculation.
 *
 * @author Tal Sitton
 */
public class FFSettings {

    /**
     * Empty FeedForwardSettings, which effectively make the FeedForwardController do nothing.
     */
    public static final FFSettings EMPTY_FF_SETTINGS = new FFSettings(
            FFController.ControlMode.LINEAR_POSITION);

    /**
     * The applied control mode
     */
    private final FFController.ControlMode controlMode;

    /**
     * The static constant
     */
    private Supplier<Double> kS;

    /**
     * The velocity constant
     */
    private Supplier<Double> kV;

    /**
     * The acceleration constant
     */
    private Supplier<Double> kA;

    /**
     * The gravity constant
     */
    private Supplier<Double> kG;

    public FFSettings(Supplier<Double> kS, Supplier<Double> kV, Supplier<Double> kA, Supplier<Double> kG,
                      FFController.ControlMode controlMode) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.controlMode = controlMode;
    }

    public FFSettings(Supplier<Double> kS, Supplier<Double> kV, Supplier<Double> kA,
                      FFController.ControlMode controlMode) {
        this(kS, kV, kA, () -> 0.0, controlMode);
    }

    public FFSettings(Supplier<Double> kV, Supplier<Double> kA,
                      FFController.ControlMode controlMode) {
        this(() -> 0.0, kV, kA, () -> 0.0, controlMode);
    }

    public FFSettings(double kS, double kV, double kA, FFController.ControlMode controlMode) {
        this(() -> kS, () -> kV, () -> kA, () -> 0.0, controlMode);
    }

    public FFSettings(double kV, double kA, FFController.ControlMode controlMode) {
        this(() -> 0.0, () -> kV, () -> kA, () -> 0.0, controlMode);
    }

    public FFSettings(double kS, double kV, double kA, double kG,
                      FFController.ControlMode controlMode) {
        this(() -> kS, () -> kV, () -> kA, () -> kG, controlMode);
    }

    public FFSettings(FFController.ControlMode controlMode) {
        this(() -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0, controlMode);
    }

    public double getkS() {
        return kS.get();
    }

    public void setkS(Supplier<Double> kS) {
        this.kS = kS;
    }

    public double getkV() {
        return kV.get();
    }

    public void setkV(Supplier<Double> kV) {
        this.kV = kV;
    }

    public double getkA() {
        return kA.get();
    }

    public void setkA(Supplier<Double> kA) {
        this.kA = kA;
    }

    public double getkG() {
        return kG.get();
    }

    public void setkG(Supplier<Double> kG) {
        this.kG = kG;
    }

    public FFController.ControlMode getControlMode() {
        return controlMode;
    }
}
