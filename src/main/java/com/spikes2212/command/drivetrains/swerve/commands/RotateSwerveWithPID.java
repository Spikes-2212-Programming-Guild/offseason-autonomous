package com.spikes2212.command.drivetrains.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.control.FFSettings;
import com.spikes2212.control.FFController;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * A command that moves a given {@link SwerveDrivetrain} to a certain angle using pid to
 * position.
 *
 * @author Gil Ein-Gar
 * @see SwerveDrivetrain
 */
public class RotateSwerveWithPID extends Command { //@TODO change the name

    private final SwerveDrivetrain drivetrain;
    private final Supplier<Double> setpoint;

    protected final PIDSettings pidSettings;
    protected final FFSettings ffSettings;
    protected final PIDController pidController;
    protected final FFController ffController;

    protected final Supplier<Double> xSpeed;
    protected final Supplier<Double> ySpeed;

    protected double lastGivenTime;
    protected double now;
    protected double lastTimeNotOnTarget;

    /**
     * Constructs a new {@link RotateModulesWithPID} command that moves the given
     * {@link SwerveDrivetrain} to a certain angle as well as on the x-axis or y-axis.
     *
     * @param drivetrain  the swerve drivetrain this command operates on
     * @param setpoint    the desired angle
     * @param xSpeed      the optional speed on the x-axis
     * @param ySpeed      the optional speed on the y-axis
     * @param pidSettings the pid settings of the given {@link SwerveDrivetrain} rotational movement
     * @param ffSettings  the feed forward settings of the given {@link SwerveDrivetrain} rotational movement
     */
    public RotateSwerveWithPID(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, Supplier<Double> xSpeed,
                               Supplier<Double> ySpeed, PIDSettings pidSettings,
                               FFSettings ffSettings) {
        this.drivetrain = drivetrain;
        this.setpoint = setpoint;
        this.pidSettings = pidSettings;
        this.ffSettings = ffSettings;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;

        pidController = new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setIZone(pidSettings.getIZone());
        pidController.setTolerance(pidSettings.getTolerance());

        ffController = new FFController(ffSettings);
        lastTimeNotOnTarget = 0;
    }

    /**
     * Constructs a new {@link RotateModulesWithPID} command that moves the given
     * {@link SwerveDrivetrain} to a certain angle.
     *
     * @param drivetrain     the swerve drivetrain this command operates on
     * @param setpoint       the desired angle
     * @param pidSettings    the pid settings of the given {@link SwerveDrivetrain} rotational movement
     * @param ffSettings     the feed forward settings of the given {@link SwerveDrivetrain} rotational movement
     * @param useVelocityPID whether the module will drive with P.I.D for the velocity
     */
    public RotateSwerveWithPID(SwerveDrivetrain drivetrain, Supplier<Double> setpoint, PIDSettings pidSettings,
                               FFSettings ffSettings, boolean useVelocityPID) {
        this(drivetrain, setpoint, () -> 0.0, () -> 0.0, pidSettings, ffSettings);
    }

    @Override
    public void initialize() {
        lastGivenTime = Timer.getFPGATimestamp();
        lastTimeNotOnTarget = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        now = Timer.getFPGATimestamp();
        ffController.setGains(ffSettings);
        pidController.setPID(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        pidController.setIZone(pidSettings.getIZone());
        pidController.setTolerance(pidSettings.getTolerance());
        drivetrain.drive(xSpeed.get(), ySpeed.get(), pidController.calculate(
                drivetrain.getAngle().getDegrees(), setpoint.get())
                + ffController.calculate(drivetrain.getAngle().getDegrees(),
                setpoint.get()), false, now - lastGivenTime);
        lastGivenTime = now;
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return pidSettings.getWaitTime() <= now - lastTimeNotOnTarget;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
