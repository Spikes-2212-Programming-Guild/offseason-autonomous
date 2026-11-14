package com.spikes2212.commandbased.swerve;

import com.spikes2212.commandbased.subsystems.DashboardedSubsystem;
import com.spikes2212.control.FFSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static java.lang.Math.abs;

/**
 * Represents a single swerve module, serving as the abstraction for one wheel’s steering and driving
 * mechanisms within a swerve drivetrain.
 *
 * @author Gil Ein-Gar
 * @see DashboardedSubsystem
 */
public abstract class SwerveModule extends DashboardedSubsystem {

    protected static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DEGREES_TO_FLIP = 180;
    private static final double MAX_DISTANCE_TO_ROTATE = 90;

    private final SmartMotorController driveMotor;
    private final SmartMotorController turnMotor;

    protected final boolean driveMotorInverted;
    protected final boolean turnMotorInverted;
    protected double absoluteEncoderOffset;
    protected final double minSpeedLimit;

    protected PIDSettings driveMotorPIDSettings;
    protected PIDSettings turnMotorPIDSettings;
    protected FFSettings driveMotorFFSettings;
    protected FFSettings turnMotorFFSettings;

    protected final boolean updateTurnPeriodically;
    protected final boolean updateDrivePeriodically;

    protected SwerveModuleState targetState = new SwerveModuleState();

    /**
     * Constructs a new instance of  {@link SwerveModule}.
     *
     * @param namespaceName         the namespace name used for the module
     * @param driveMotor            the motor that is used for turning the wheel in the pitch axis
     * @param turnMotor             the motor that is used fot turning the wheel in the yaw axis
     * @param driveMotorInverted    whether the drive motor is inverted or not
     * @param turnMotorInverted     whether the turn motor is inverted ot not
     * @param absoluteEncoderOffset absolute encoder offset while wheel is parallel to the track
     * @param driveMotorPIDSettings the drive motor pid settings
     * @param turnMotorPIDSettings  the turn motor pid settings
     * @param driveMotorFFSettings  the drive motor feed forward settings
     * @param turnMotorFFSettings   the turn motor feed forward settings
     * @param minSpeedLimit         a speed limit intended for dealing with stick drift
     */
    public SwerveModule(String namespaceName, SmartMotorController driveMotor, SmartMotorController turnMotor,
                        boolean driveMotorInverted, boolean turnMotorInverted, double absoluteEncoderOffset,
                        PIDSettings driveMotorPIDSettings, PIDSettings turnMotorPIDSettings,
                        FFSettings driveMotorFFSettings,
                        FFSettings turnMotorFFSettings, double minSpeedLimit,
                        boolean updateDrivePeriodically, boolean updateTurnPeriodically) {
        super(namespaceName);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnMotorInverted = turnMotorInverted;
        this.driveMotorInverted = driveMotorInverted;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.driveMotorPIDSettings = driveMotorPIDSettings;
        this.turnMotorPIDSettings = turnMotorPIDSettings;
        this.driveMotorFFSettings = driveMotorFFSettings;
        this.turnMotorFFSettings = turnMotorFFSettings;
        this.minSpeedLimit = minSpeedLimit;
        this.updateDrivePeriodically = updateDrivePeriodically;
        this.updateTurnPeriodically = updateTurnPeriodically;
    }

    /**
     * Configures drive motor conversion factors that translate encoder rotations
     * into real-world distance and speed measurements.
     */
    protected abstract void configureDriveController();

    /**
     * Configures turn motor conversion factors that convert motor rotations
     * into angular position and velocity values in degrees.
     */
    protected abstract void configureTurnController();

    /**
     * Defines the contract for configuring an absolute encoder.
     * Implementations should set direction, offset, and calibration parameters
     * to align encoder readings with the system’s mechanics.
     */
    protected abstract void configureAbsoluteEncoder();

    /**
     * Sets the desired module state.
     *
     * @param targetState the desired state of the module
     */
    public void setTargetState(SwerveModuleState targetState) {
        targetState = optimize(targetState, getRelativeModuleAngle());
        if (Math.abs(targetState.speedMetersPerSecond) <= minSpeedLimit) {
            stop();
            return;
        }
        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond);
    }

    /**
     * Normalize the relative angle of the {@link SwerveModule} to be in the 0-360 scope.
     *
     * @param currentAngle the current angle of the {@link SwerveModule} in degrees
     * @param desiredAngle the desired angle of the {@link SwerveModule} in degrees
     * @return the normalized desired angle of the {@link SwerveModule} in degrees
     */
    private double normalizeAngleRelativeToRelativeEncoder(double currentAngle, double desiredAngle) {
        int rotations = (int) (currentAngle / DEGREES_IN_ROTATIONS);
        if (currentAngle < 0) rotations--;
        desiredAngle += rotations * DEGREES_IN_ROTATIONS;
        return desiredAngle;
    }

    /**
     * Optimizes the {@link SwerveModule} angle
     *
     * @param targetState  the desired state of the {@link SwerveModule}
     * @param currentAngle the current angle of the {@link SwerveModule} in degrees
     * @return the optimized {@link SwerveModuleState}
     */
    private SwerveModuleState optimize(SwerveModuleState targetState, double currentAngle) {
        double desiredAngle = normalizeAngleRelativeToRelativeEncoder(currentAngle, targetState.angle.getDegrees());
        while (abs(desiredAngle - currentAngle) > MAX_DISTANCE_TO_ROTATE) {
            if (desiredAngle - currentAngle > 0) {
                desiredAngle -= DEGREES_TO_FLIP;
            } else {
                desiredAngle += DEGREES_TO_FLIP;
            }
            targetState.speedMetersPerSecond *= -1;
        }
        return new SwerveModuleState(targetState.speedMetersPerSecond, Rotation2d.fromDegrees(desiredAngle));
    }

    /**
     * Sets the desired module angle.
     *
     * @param targetAngle the desired angle of the module
     */
    public void setTargetAngle(Rotation2d targetAngle) {
        targetState.angle = targetAngle;
        turnMotor.pidSet(UnifiedControlMode.POSITION, targetAngle.getDegrees(), turnMotorPIDSettings,
                turnMotorFFSettings, updateTurnPeriodically);
    }

    /**
     * Sets the desired module velocity.
     *
     * @param targetVelocity the desired velocity of the module
     */
    public void setTargetVelocity(double targetVelocity) {
        this.targetState.speedMetersPerSecond = targetVelocity;
        driveMotor.pidSet(UnifiedControlMode.VELOCITY, targetVelocity, driveMotorPIDSettings,
                driveMotorFFSettings, updateDrivePeriodically);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDrivingVelocity(), Rotation2d.fromDegrees(getRelativeModuleAngle()));
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public double getRelativeModuleAngle() {
        return turnMotor.getPosition();
    }

    /**
     * @return the current module velocity in meters per second
     */
    public double getDrivingVelocity() {
        return driveMotor.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), getAbsoluteModuleAngle());
    }

    public PIDSettings getDriveMotorPIDSettings() {
        return driveMotorPIDSettings;
    }

    public PIDSettings getTurnMotorPIDSettings() {
        return turnMotorPIDSettings;
    }

    public FFSettings getDriveMotorFeedForwardSettings() {
        return driveMotorFFSettings;
    }

    public FFSettings getTurnMotorFeedForwardSettings() {
        return turnMotorFFSettings;
    }

    public SmartMotorController getTurnMotor() {
        return turnMotor;
    }

    public SmartMotorController getDriveMotor() {
        return driveMotor;
    }

    /**
     * Resets the relative encoder according to the absolute encoder.
     */
    public void resetRelativeEncoder() {
        turnMotor.setPosition(getAbsoluteModuleAngle().getDegrees());
    }

    /**
     * @return the absolute module angle using an absolute encoder.
     */
    protected abstract Rotation2d getAbsoluteModuleAngle();

    /**
     * Stops the drive and turn motors of the module.
     */
    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
