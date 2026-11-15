
package com.spikes2212.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class RootNamespace implements Namespace, AutoCloseable {

    protected final NetworkTable table;
    private final String name;

    private final Map<String, PublishedField<?>> tunableFields = new HashMap<>();
    private final Map<String, Publisher> publishers = new HashMap<>();
    private final Map<String, Subscriber> subscribers = new HashMap<>();
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
            DriverStation.reportError("A field named '" + name + "' already exists in namespace '"
                    + this.name + "'.", true);
        }
    }

    private <T, P extends Publisher, S extends Subscriber> Supplier<T> registerTunable(
            String name, P publisher, S subscriber, T initialValue, Consumer<T> setDefault, Supplier<T> getter
    ) {
        setDefault.accept(initialValue);
        publishers.put(name, publisher);
        subscribers.put(name, subscriber);
        return getter;
    }

    @Override
    public Supplier<Double> addTunableDouble(String name, double initialValue) {
        ensureAvailableName(name);
        var topic = table.getDoubleTopic(name);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber::get);
    }

    @Override
    public Supplier<Boolean> addTunableBoolean(String name, boolean initialValue) {
        ensureAvailableName(name);
        var topic = table.getBooleanTopic(name);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber::get);
    }

    @Override
    public Supplier<Long> addTunableLong(String name, long initialValue) {
        ensureAvailableName(name);
        var topic = table.getIntegerTopic(name);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber::get);
    }

    @Override
    public Supplier<String> addTunableString(String name, String initialValue) {
        ensureAvailableName(name);
        var topic = table.getStringTopic(name);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber);
    }

    public Supplier<double[]> addTunableDoubleArray(String name, double[] initialValue) {
        ensureAvailableName(name);
        var topic = table.getDoubleArrayTopic(name);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber);
    }

    public Supplier<boolean[]> addTunableBooleanArray(String name, boolean[] initialValue) {
        ensureAvailableName(name);
        var topic = table.getBooleanArrayTopic(name);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber);
    }

    public Supplier<long[]> addTunableLongArray(String name, long[] initialValue) {
        ensureAvailableName(name);
        var topic = table.getIntegerArrayTopic(name);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber);
    }

    public Supplier<String[]> addTunableStringArray(String name, String[] initialValue) {
        ensureAvailableName(name);
        var topic = table.getStringArrayTopic(name);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber);
    }

    public <T> Supplier<T> addTunableStruct(String name, Struct<T> struct, T initialValue) {
        ensureAvailableName(name);
        var topic = table.getStructTopic(name, struct);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber);
    }

    public <T> Supplier<T[]> addTunableStructArray(String name, Struct<T> struct, T[] initialValue) {
        ensureAvailableName(name);
        var topic = table.getStructArrayTopic(name, struct);
        var publisher = topic.publish();
        var subscriber = topic.subscribe(initialValue);
        return registerTunable(name, publisher, subscriber, initialValue, publisher::setDefault, subscriber);
    }

    @Override
    public void publishDouble(String name, Supplier<Double> value) {
        ensureAvailableName(name);
        var publisher = table.getDoubleTopic(name).publish();
        publish(name, publisher, value, publisher::set);
    }

    @Override
    public void publishBoolean(String name, Supplier<Boolean> value) {
        ensureAvailableName(name);
        var publisher = table.getBooleanTopic(name).publish();
        publish(name, publisher, value, publisher::set);
    }

    @Override
    public void publishLong(String name, Supplier<Long> value) {
        ensureAvailableName(name);
        var publisher = table.getIntegerTopic(name).publish();
        publish(name, publisher, value, publisher::set);
    }

    @Override
    public void publishString(String name, Supplier<String> value) {
        ensureAvailableName(name);
        var publisher = table.getStringTopic(name).publish();
        publish(name, publisher, value, publisher::set);
    }

    private <T> void publish(String name, Publisher publisher, Supplier<T> valueSupplier, Consumer<T> valuePublisher) {
        publishers.put(name, publisher);
        tunableFields.put(name, new PublishedField<>(valueSupplier, valuePublisher));
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
        return sendables.get(name);
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

    public void update() {
        tunableFields.values().forEach(PublishedField::update);
        sendables.values().forEach(SendableRegistry::update);
    }

    @Override
    public void close() {
        publishers.values().forEach(Publisher::close);
        subscribers.values().forEach(Subscriber::close);
        publishers.clear();
        subscribers.clear();
        tunableFields.clear();
        sendables.clear();
    }
}
