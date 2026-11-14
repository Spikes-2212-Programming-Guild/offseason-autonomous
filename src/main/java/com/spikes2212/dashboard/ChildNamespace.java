
package com.spikes2212.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

public class ChildNamespace extends RootNamespace {

    protected final Namespace parent;
    protected final String separator;

    public ChildNamespace(String name, Namespace parent) {
        this(name, parent, "/");
    }

    public ChildNamespace(String name, Namespace parent, String separator) {
        super(name);
        this.parent = java.util.Objects.requireNonNull(parent);
        this.separator = java.util.Objects.requireNonNull(separator);
    }

    private String getFieldName(String name) {
        return getName() + separator + name;
    }

    @Override
    public String getName() {
        return parent.getName() + separator + super.getName();
    }

    @Override
    public ChildNamespace addChild(String name) {
        return new ChildNamespace(name, this, separator);
    }

    @Override
    public Supplier<Double> addTunableDouble(String name, double initialValue) {
        return parent.addTunableDouble(getFieldName(name), initialValue);
    }

    @Override
    public Supplier<Boolean> addTunableBoolean(String name, boolean initialValue) {
        return parent.addTunableBoolean(getFieldName(name), initialValue);
    }

    @Override
    public Supplier<Long> addTunableLong(String name, long initialValue) {
        return parent.addTunableLong(getFieldName(name), initialValue);
    }

    @Override
    public Supplier<String> addTunableString(String name, String initialValue) {
        return parent.addTunableString(getFieldName(name), initialValue);
    }

    @Override
    public void publishDouble(String name, Supplier<Double> value) {
        parent.publishDouble(getFieldName(name), value);
    }

    @Override
    public void publishBoolean(String name, Supplier<Boolean> value) {
        parent.publishBoolean(getFieldName(name), value);
    }

    @Override
    public void publishLong(String name, Supplier<Long> value) {
        parent.publishLong(getFieldName(name), value);
    }

    @Override
    public void publishString(String name, Supplier<String> value) {
        parent.publishString(getFieldName(name), value);
    }

    @Override
    public void publishSendable(String name, Sendable sendable) {
        parent.publishSendable(getFieldName(name), sendable);
    }

    @Override
    public void publishCommand(String name, Command command, boolean runOnDisable) {
        parent.publishCommand(getFieldName(name), command, runOnDisable);
    }

    @Override
    public void publishCommand(String name, Command command) {
        parent.publishCommand(getFieldName(name), command);
    }

    @Override
    public Sendable getSendable(String name) {
        return parent.getSendable(getFieldName(name));
    }

    @Override
    public String getString(String name, String defaultValue) {
        return parent.getString(getFieldName(name), defaultValue);
    }

    @Override
    public double getDouble(String name, double defaultValue) {
        return parent.getDouble(getFieldName(name), defaultValue);
    }

    @Override
    public long getLong(String name, long defaultValue) {
        return parent.getLong(getFieldName(name), defaultValue);
    }

    @Override
    public boolean getBoolean(String name, boolean defaultValue) {
        return parent.getBoolean(getFieldName(name), defaultValue);
    }

    @Override
    public void update() {
        // Resources are owned and updated by the root namespace.
    }
}
