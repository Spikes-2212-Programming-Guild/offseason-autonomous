package com.spikes2212.commandbased.swerve;

import com.spikes2212.commandbased.subsystems.DashboardedSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Arrays;

/**
 * Represents a swerve drivetrain that uses vector calculations to combine the robot’s desired translation
 * and rotation, determining each wheel’s speed and direction for smooth, omnidirectional movement.
 *
 * @author Gil Ein-Gar
 * @see DashboardedSubsystem
 */
public abstract class SwerveDrivetrain extends DashboardedSubsystem {

    private static final double DELTA_TIME = 0.02;

    protected final SwerveModule frontLeftModule;
    protected final SwerveModule frontRightModule;
    protected final SwerveModule backLeftModule;
    protected final SwerveModule backRightModule;

    protected final double halfDrivetrainTrackWidth;
    protected final double halfDrivetrainTrackLength;
    protected final double maxPossibleVelocity;
    protected final SwerveDriveKinematics kinematics;

    /**
     * Constructs a new instance of {@link SwerveDrivetrain}.
     *
     * @param namespaceName         the namespace name for the drivetrain
     * @param frontLeftModule       the front left module using {@link SwerveModule}
     * @param frontRightModule      the front right module using {@link SwerveModule}
     * @param backLeftModule        the back left module using {@link SwerveModule}
     * @param backRightModule       the back right module using{@link SwerveModule}
     * @param drivetrainTrackWidth  the width of the drivetrain
     * @param drivetrainTrackLength the length of the drivetrain
     * @param maxPossibleVelocity   the maximum possible velocity of the drive motors
     */
    public SwerveDrivetrain(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule,
                            SwerveModule backLeftModule, SwerveModule backRightModule, double drivetrainTrackWidth,
                            double drivetrainTrackLength, double maxPossibleVelocity) {
        super(namespaceName);
        this.frontLeftModule = frontLeftModule;
        this.frontRightModule = frontRightModule;
        this.backLeftModule = backLeftModule;
        this.backRightModule = backRightModule;
        this.halfDrivetrainTrackWidth = drivetrainTrackWidth / 2;
        this.halfDrivetrainTrackLength = drivetrainTrackLength / 2;
        this.maxPossibleVelocity = maxPossibleVelocity;
        kinematics = calculateKinematics();
        resetRelativeEncoders();
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
        drive(xSpeed, ySpeed, rotationSpeed, true, DELTA_TIME);
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldRelative, double timeStep) {
        drive(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed), fieldRelative, timeStep);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, double timeStep) {
        if (fieldRelative) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, timeStep));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxPossibleVelocity);
        setTargetModuleStates(states);
    }

    protected void setTargetModuleStates(SwerveModuleState[] targetModuleStates) {
        SwerveModule[] modules = getModules();
        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(targetModuleStates[i]);
        }
    }

    public abstract Rotation2d getAngle();

    public abstract void resetAngleSensor();

    /**
     * Resets the field relativity to be the same as the robot relativity.
     */
    public void resetFieldRelativity() {
        resetAngleSensor();
    }

    protected SwerveDriveKinematics calculateKinematics() {
        Translation2d frontLeftWheelDistanceFromCenter =
                new Translation2d(halfDrivetrainTrackWidth, halfDrivetrainTrackLength);
        Translation2d frontRightWheelDistanceFromCenter =
                new Translation2d(halfDrivetrainTrackWidth, -halfDrivetrainTrackLength);
        Translation2d backLeftWheelDistanceFromCenter =
                new Translation2d(-halfDrivetrainTrackWidth, halfDrivetrainTrackLength);
        Translation2d backRightWheelDistanceFromCenter =
                new Translation2d(-halfDrivetrainTrackWidth, -halfDrivetrainTrackLength);

        return new SwerveDriveKinematics(frontLeftWheelDistanceFromCenter,
                frontRightWheelDistanceFromCenter, backLeftWheelDistanceFromCenter,
                backRightWheelDistanceFromCenter);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(getCurrentModuleStates());
    }

    public SwerveModule getFrontLeftModule() {
        return frontLeftModule;
    }

    public SwerveModule getFrontRightModule() {
        return frontRightModule;
    }

    public SwerveModule getBackLeftModule() {
        return backLeftModule;
    }

    public SwerveModule getBackRightModule() {
        return backRightModule;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return Arrays.stream(getModules()).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    public SwerveModuleState[] getCurrentModuleStates() {
        return Arrays.stream(getModules()).map(SwerveModule::getCurrentState).toArray(SwerveModuleState[]::new);
    }

    public SwerveModuleState[] getTargetModuleStates() {
        return Arrays.stream(getModules()).map(SwerveModule::getTargetState).toArray(SwerveModuleState[]::new);
    }

    /**
     * Resets the relative encoders of every module according to their absolute encoder.
     */
    public void resetRelativeEncoders() {
        Arrays.stream(getModules()).forEach(SwerveModule::resetRelativeEncoder);
    }

    public SwerveModule[] getModules() {
        return new SwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};
    }

    public void stop() {
        Arrays.stream(getModules()).forEach(SwerveModule::stop);
    }
}
