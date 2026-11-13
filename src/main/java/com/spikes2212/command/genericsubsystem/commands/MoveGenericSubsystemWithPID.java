package com.spikes2212.command.genericsubsystem.commands;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.control.FFController;
import com.spikes2212.control.FFSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * This command moves a {@link GenericSubsystem} according to a {@link Supplier}
 * or a constant speed until it reaches its target or can't move anymore.
 *
 * @author Yuval Levy
 * @see GenericSubsystem
 */
public class MoveGenericSubsystemWithPID extends Command {

    protected final GenericSubsystem subsystem;
    protected final Supplier<Double> source;
    protected final PIDSettings pidSettings;
    protected final FFSettings ffSettings;
    protected final Supplier<Double> setpoint;
    protected final Supplier<Double> acceleration;

    /**
     * An object that makes the necessary calculations for the PID control loop.
     */
    protected final PIDController pidController;

    /**
     * An object that makes the necessary calculations for the feed forward control loop.
     */
    protected final FFController ffController;

    /**
     * The last time the subsystem didn't reach the target.
     */
    private double lastTimeNotOnTarget;

    /**
     * Constructs a new {@link MoveGenericSubsystemWithPID} command that moves the given
     * {@link GenericSubsystem} towards a setpoint given from a Double {@link Supplier}.
     *
     * @param subsystem           the subsystem this command operates on
     * @param setpoint            the Double {@link Supplier} supplying the setpoint
     * @param source              the Double {@link Supplier} supplying the current position
     * @param pidSettings         the pid constants used for calculating the move value for each iteration
     * @param ffSettings the feed forward constants used for calculating the move value
     */
    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, Supplier<Double> setpoint, Supplier<Double> source,
                                       Supplier<Double> acceleration, PIDSettings pidSettings,
                                       FFSettings ffSettings) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.pidSettings = pidSettings;
        this.ffSettings = ffSettings;
        this.setpoint = setpoint;
        this.acceleration = acceleration;
        this.source = source;
        this.ffController = new FFController(ffSettings.getkS(), ffSettings.getkV(),
                ffSettings.getkA(), ffSettings.getkG(), ffSettings.getControlMode());
        this.pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, double setpoint, double source,
                                       Supplier<Double> acceleration, PIDSettings pidSettings,
                                       FFSettings ffSettings) {
        this(subsystem, () -> setpoint, () -> source, acceleration, pidSettings, ffSettings);
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, Supplier<Double> setpoint, Supplier<Double> source,
                                       PIDSettings pidSettings) {
        this(subsystem, setpoint, source, () -> 0.0, pidSettings, FFSettings.EMPTY_FF_SETTINGS);
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, double setpoint, double source,
                                       PIDSettings pidSettings) {
        this(subsystem, () -> setpoint, () -> source, pidSettings);
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, Supplier<Double> setpoint, Supplier<Double> source,
                                       PIDSettings pidSettings, FFSettings ffSettings) {
        this(subsystem, setpoint, source, () -> 0.0, pidSettings, ffSettings);
    }

    public MoveGenericSubsystemWithPID(GenericSubsystem subsystem, double setpoint, double source,
                                       PIDSettings pidSettings, FFSettings ffSettings) {
        this(subsystem, () -> setpoint, () -> source, () -> 0.0, pidSettings, ffSettings);
    }

    protected double calculatePIDAndFFValues() {
        pidController.setTolerance(pidSettings.getTolerance());
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setIZone(pidSettings.getIZone());
        ffController.setGains(ffSettings);

        double pidValue = pidController.calculate(source.get(), setpoint.get());
        double svagValue = ffController.calculate(source.get(), setpoint.get(), acceleration.get());
        return pidValue + svagValue;
    }

    @Override
    public void execute() {
        subsystem.move(calculatePIDAndFFValues());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= pidSettings.getWaitTime();
    }
}
