
package com.spikes2212.dashboard;

import com.spikes2212.control.FFSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

public interface Namespace {

    default ChildNamespace addChild(String name) {
        return new ChildNamespace(name, this);
    }

    String getName();

    Supplier<Boolean> addTunableBoolean(String name, boolean initialValue);

    Supplier<Double> addTunableDouble(String name, double initialValue);

    Supplier<Long> addTunableLong(String name, long initialValue);

    Supplier<String> addTunableString(String name, String initialValue);

    void publishSendable(String name, Sendable sendable);

    default void publishCommand(String name, Command command, boolean runOnDisable) {
        publishSendable(name, command.ignoringDisable(runOnDisable));
    }

    default void publishCommand(String name, Command command) {
        publishCommand(name, command, false);
    }

    default void publishRunnable(String name, Runnable runnable) {
        publishCommand(name, new InstantCommand(runnable));
    }

    void publishBoolean(String name, Supplier<Boolean> value);

    void publishString(String name, Supplier<String> value);

    void publishLong(String name, Supplier<Long> value);

    void publishDouble(String name, Supplier<Double> value);

    Sendable getSendable(String name);

    String getString(String name, String defaultValue);

    default String getString(String name) {
        return getString(name, "");
    }

    double getDouble(String name, double defaultValue);

    default double getDouble(String name) {
        return getDouble(name, 0);
    }

    long getLong(String name, long defaultValue);

    default long getLong(String name) {
        return getLong(name, 0);
    }

    boolean getBoolean(String name, boolean defaultValue);

    default boolean getBoolean(String name) {
        return getBoolean(name, false);
    }

    default PIDSettings addPIDNamespace(String name, PIDSettings initial) {
        ChildNamespace child = addChild(name + " pid");
        Supplier<Double> kP = child.addTunableDouble(name + " kP", initial.getkP());
        Supplier<Double> kI = child.addTunableDouble(name + " kI", initial.getkI());
        Supplier<Double> kD = child.addTunableDouble(name + " kD", initial.getkD());
        Supplier<Double> iZone = child.addTunableDouble(name + " i zone", initial.getIZone());
        Supplier<Double> tolerance = child.addTunableDouble(name + " tolerance", initial.getTolerance());
        Supplier<Double> waitTime = child.addTunableDouble(name + " wait time", initial.getWaitTime());
        return new PIDSettings(kP, kI, kD, iZone, tolerance, waitTime);
    }

    default FFSettings addFFNamespace(String name, FFSettings initial) {
        ChildNamespace child = addChild(name + " ff");
        Supplier<Double> kS = child.addTunableDouble(name + " kS", initial.getkS());
        Supplier<Double> kV = child.addTunableDouble(name + " kV", initial.getkV());
        Supplier<Double> kA = child.addTunableDouble(name + " kA", initial.getkA());
        Supplier<Double> kG = child.addTunableDouble(name + " kG", initial.getkG());
        return new FFSettings(kS, kV, kA, kG, initial.getControlMode());
    }

    void update();
}
