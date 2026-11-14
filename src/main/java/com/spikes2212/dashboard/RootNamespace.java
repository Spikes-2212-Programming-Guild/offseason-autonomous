
package com.spikes2212.dashboard;

import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class RootNamespace implements Namespace {

    protected final NetworkTable table;
    private final String name;

    private final Map<String, PublishedField<?>> publishers = new HashMap<>();
    private final Map<String, Sendable> sendables = new HashMap<>();

    public RootNamespace(String name) {
        this.name = Objects.requireNonNull(name);
        this.table = NetworkTableInstance.getDefault().getTable(name);
    }

    @Override
    public String getName() {
        return name;
    }

    private void ensureAvailableName(String name) {
        if (name == null || name.isBlank())
            throw new IllegalArgumentException("Name cannot be null or blank.");

        if (publishers.containsKey(name) || sendables.containsKey(name)) {
            throw new IllegalArgumentException(
                    "A resource named '" + name + "' already exists in namespace '" + this.name + "'."
            );
        }
    }

    @Override
    public Supplier<Double> addTunableDouble(String name, double initialValue) {
        return () -> (table.getDoubleTopic(name).subscribe(initialValue)).get();
    }

    @Override
    public Supplier<Boolean> addTunableBoolean(String name, boolean initialValue) {
        return () -> (table.getBooleanTopic(name).subscribe(initialValue)).get();
    }

    @Override
    public Supplier<Long> addTunableLong(String name, long initialValue) {
        return () -> (table.getIntegerTopic(name).subscribe(initialValue)).get();
    }

    @Override
    public Supplier<String> addTunableString(String name, String initialValue) {
        return table.getStringTopic(name).subscribe(initialValue);
    }

    @Override
    public void publishDouble(String name, Supplier<Double> value) {
        ensureAvailableName(name);
        DoublePublisher publisher = table.getDoubleTopic(name).publish();
        publish(name, value, publisher::set);
    }

    @Override
    public void publishBoolean(String name, Supplier<Boolean> value) {
        ensureAvailableName(name);
        BooleanPublisher publisher = table.getBooleanTopic(name).publish();
        publish(name, value, publisher::set);
    }

    @Override
    public void publishLong(String name, Supplier<Long> value) {
        ensureAvailableName(name);
        IntegerPublisher publisher = table.getIntegerTopic(name).publish();
        publish(name, value, publisher::set);
    }

    @Override
    public void publishString(String name, Supplier<String> value) {
        ensureAvailableName(name);
        StringPublisher publisher = table.getStringTopic(name).publish();
        publish(name, value, publisher::set);
    }

    private <T> void publish(String name, Supplier<T> valueSupplier, Consumer<T> valuePublisher) {
        publishers.put(name, new PublishedField<>(valueSupplier, valuePublisher));
    }

    @Override
    public void publishSendable(String name, Sendable sendable) {
        ensureAvailableName(name);
        Objects.requireNonNull(sendable);
        NetworkTable dataTable = table.getSubTable(name);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable);
        SendableRegistry.publish(sendable, builder);
        builder.startListeners();
        dataTable.getEntry(".name").setString(name);
        sendables.put(name, sendable);
    }

    @Override
    public void publishCommand(String name, Command command) {
        publishSendable(name, command);
    }

    @Override
    public Sendable getSendable(String name) {
        return sendables.getOrDefault(name, null);
    }

    public boolean getBoolean(String name, boolean defaultValue) {
        return table.getEntry(name).getBoolean(defaultValue);
    }

    public double getDouble(String name, double defaultValue) {
        return table.getEntry(name).getDouble(defaultValue);
    }

    public long getLong(String name, long defaultValue) {
        return table.getEntry(name).getInteger(defaultValue);
    }

    public String getString(String name, String defaultValue) {
        return table.getEntry(name).getString(defaultValue);
    }

    @Override
    public void update() {
        publishers.values().forEach(PublishedField::update);
        sendables.values().forEach(SendableRegistry::update);
    }
}
