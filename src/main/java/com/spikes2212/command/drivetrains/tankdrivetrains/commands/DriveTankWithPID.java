package com.spikes2212.command.drivetrains.tankdrivetrains.commands;

import com.spikes2212.command.drivetrains.tankdrivetrains.TankDrivetrain;
import com.spikes2212.control.FFController;
import com.spikes2212.control.FFSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * A command that moves a {@link TankDrivetrain} using two PID loops, one for each side.
 *
 * @author Yuval Levy
 * @see TankDrivetrain
 */
public class DriveTankWithPID extends Command {

    /**
     * The drivetrain this command operates on.
     */
    protected final TankDrivetrain drivetrain;

    /**
     * The PID Settings for the PID loop operating on the left side of the drivetrain.
     */
    protected final PIDSettings leftPIDSettings;

    /**
     * The PID Settings for the PID loop operating on the right side of the drivetrain.
     */
    protected final PIDSettings rightPIDSettings;

    /**
     * The FeedForwards Settings of the FeedForward loop operating on the left side of the drivetrain.
     */
    protected final FFSettings leftFFSettings;

    /**
     * The FeedForwards Settings of the FeedForward loop operating on the right side of the drivetrain.
     */
    protected final FFSettings rightFFSettings;

    /**
     * The PID Controller of the PID loop operating on the left side of the drivetrain.
     */
    protected final PIDController leftPIDController;

    /**
     * The PID Controller of the PID loop operating on the right side of the drivetrain.
     */
    protected final PIDController rightPIDController;

    /**
     * The setpoint the left side of the drivetrain should reach.
     */
    protected final Supplier<Double> leftSetpoint;

    /**
     * The setpoint the right side of the drivetrain should reach.
     */
    protected final Supplier<Double> rightSetpoint;

    /**
     * The acceleration the left side of the drivetrain should reach.
     */
    protected final Supplier<Double> leftAcceleration;

    /**
     * The acceleration the right side of the drivetrain should reach.
     */
    protected final Supplier<Double> rightAcceleration;

    /**
     * How far the left side of the drivetrain drove.
     */
    protected final Supplier<Double> leftSource;

    /**
     * How far the right side of the drivetrain drove.
     */
    protected final Supplier<Double> rightSource;

    /**
     * The FeedForwards Controller of the FeedForward loop operating on the left side of the drivetrain.
     */
    protected final FFController leftFFController;

    /**
     * The FeedForwards Controller of the FeedForward loop operating on the right side of the drivetrain.
     */
    protected final FFController rightFFController;

    /**
     * The last time the left side of the drivetrain was not within its target zone.
     */
    private double leftLastTimeNotOnTarget;

    /**
     * The last time the right side of the drivetrain was not within its target zone.
     */
    private double rightLastTimeNotOnTarget;

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, Supplier<Double> leftAcceleration,
                            Supplier<Double> rightAcceleration, FFSettings leftFFSettings,
                            FFSettings rightFFSettings) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.leftPIDSettings = leftPIDSettings;
        this.leftPIDController = new PIDController(leftPIDSettings.getkP(), leftPIDSettings.getkI(),
                leftPIDSettings.getkD());
        this.rightPIDSettings = rightPIDSettings;
        this.rightPIDController = new PIDController(rightPIDSettings.getkP(), rightPIDSettings.getkI(),
                rightPIDSettings.getkD());
        this.leftSetpoint = leftSetpoint;
        this.rightSetpoint = rightSetpoint;
        this.leftSource = leftSource;
        this.rightSource = rightSource;
        this.leftAcceleration = leftAcceleration;
        this.rightAcceleration = rightAcceleration;
        this.leftPIDController.setSetpoint(leftSetpoint.get());
        this.rightPIDController.setSetpoint(rightSetpoint.get());
        this.leftFFSettings = leftFFSettings;
        this.rightFFSettings = rightFFSettings;
        this.leftFFController = new FFController(leftFFSettings.getkS(),
                leftFFSettings.getkV(), leftFFSettings.getkA(), leftFFSettings.getkG(),
                leftFFSettings.getControlMode());
        this.rightFFController = new FFController(rightFFSettings.getkS(),
                rightFFSettings.getkV(), rightFFSettings.getkA(), rightFFSettings.getkG(),
                rightFFSettings.getControlMode());
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint,
                            Supplier<Double> leftSource, Supplier<Double> rightSource) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, leftSetpoint, rightSetpoint, leftSource, rightSource,
                () -> 0.0, () -> 0.0, FFSettings.EMPTY_FF_SETTINGS,
                FFSettings.EMPTY_FF_SETTINGS);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, Supplier<Double> leftAcceleration,
                            Supplier<Double> rightAcceleration, FFSettings leftFFSettings,
                            FFSettings rightFFSettings) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, () -> leftSetpoint, () -> rightSetpoint,
                leftAcceleration, rightAcceleration, leftSource, rightSource, leftFFSettings,
                rightFFSettings);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, () -> leftSetpoint, () -> rightSetpoint, () -> 0.0,
                () -> 0.0, leftSource, rightSource, FFSettings.EMPTY_FF_SETTINGS,
                FFSettings.EMPTY_FF_SETTINGS);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, FFSettings leftFFSettings,
                            FFSettings rightFFSettings) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, leftSetpoint, rightSetpoint, leftSource, rightSource,
                () -> 0.0, () -> 0.0, leftFFSettings, rightFFSettings);
    }

    public DriveTankWithPID(TankDrivetrain drivetrain, PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                            double leftSetpoint, double rightSetpoint, Supplier<Double> leftSource,
                            Supplier<Double> rightSource, FFSettings leftFFSettings,
                            FFSettings rightFFSettings) {
        this(drivetrain, leftPIDSettings, rightPIDSettings, () -> leftSetpoint, () -> rightSetpoint,
                () -> 0.0, () -> 0.0, leftSource, rightSource, leftFFSettings,
                rightFFSettings);
    }

    @Override
    public void execute() {
        leftPIDController.setSetpoint(leftSetpoint.get());
        rightPIDController.setSetpoint(rightSetpoint.get());
        leftPIDController.setTolerance(leftPIDSettings.getTolerance());
        rightPIDController.setTolerance(rightPIDSettings.getTolerance());
        leftPIDController.setPID(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD());
        rightPIDController.setPID(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD());
        leftPIDController.setIZone(leftPIDSettings.getIZone());
        rightPIDController.setIZone(rightPIDSettings.getIZone());
        leftFFController.setGains(leftFFSettings);
        rightFFController.setGains(rightFFSettings);
        drivetrain.tankDrive((leftPIDController.calculate(leftSource.get()) +
                        leftFFController.calculate(leftSource.get(), leftSetpoint.get(),
                                leftAcceleration.get())),
                rightPIDController.calculate(rightSource.get()) +
                        rightFFController.calculate(rightSource.get(), rightSetpoint.get(),
                                rightAcceleration.get()));
    }

    @Override
    public boolean isFinished() {
        if (!leftPIDController.atSetpoint()) {
            leftLastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        if (!rightPIDController.atSetpoint()) {
            rightLastTimeNotOnTarget = Timer.getFPGATimestamp();
        }

        return Timer.getFPGATimestamp() - leftLastTimeNotOnTarget >= leftPIDSettings.getWaitTime()
                && Timer.getFPGATimestamp() - rightLastTimeNotOnTarget >= rightPIDSettings.getWaitTime();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
