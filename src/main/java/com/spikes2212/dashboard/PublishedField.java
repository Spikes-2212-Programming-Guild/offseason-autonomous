package com.spikes2212.dashboard;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * a class that represents a field published through network tables.
 * it optimizes the update process by only updating the network table if the value changed
 */
public class PublishedField<T> {

    private final Supplier<T> valueSupplier;
    private final Consumer<T> valuePublisher;

    private T lastPublishedValue;
    private boolean hasPublished;

    public PublishedField(Supplier<T> valueSupplier, Consumer<T> valuePublisher) {
        this.valueSupplier = valueSupplier;
        this.valuePublisher = valuePublisher;
    }

    public void update() {
        T currentValue = valueSupplier.get();

        if (hasPublished && Objects.equals(currentValue, lastPublishedValue))
            return;

        valuePublisher.accept(currentValue);

        lastPublishedValue = currentValue;
        hasPublished = true;
    }
}