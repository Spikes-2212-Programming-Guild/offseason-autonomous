package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.swerve.subsystem.Swerve;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class AutonomousContainer {

    private static final RobotConfig CONFIG = getRobotConfig();

    private final Swerve drivetrain;

    private static final RootNamespace namespace = new RootNamespace("autonomouss");

    private static final PathConstraints pathConstraints = new PathConstraints(5.1,
            2, 8, 2);

    private static final double ROBOT_POSE_LATENCY = 0;
    private static final double TIME_STEP = 0.02;
    //    private static final double PID_TO_POSE_TIMEOUT = 0.1;
    private static final double FF_SCALER = 1;

    private final PIDController xPidController;
    private final PIDController yPidController;
    private final ProfiledPIDController rotationalPidController;


    private static final PIDSettings X_CONTROLLER_SETTINGS =
            namespace.addPIDNamespace("x controller settings", PIDSettings.EMPTY_PID_SETTINGS);
    private static final PIDSettings Y_CONTROLLER_SETTINGS =
            namespace.addPIDNamespace("y controller settings", PIDSettings.EMPTY_PID_SETTINGS);
    private static final PIDSettings ROTATIONAL_CONTROLLER_SETTINGS =
            namespace.addPIDNamespace("rotational controller settings", PIDSettings.EMPTY_PID_SETTINGS);

    private Pose2d pathplannerTargetPose;

    public AutonomousContainer(Swerve drivetrain) {
        this.drivetrain = drivetrain;

        xPidController = buildPIDControllerFromSettings(X_CONTROLLER_SETTINGS);
        yPidController = buildPIDControllerFromSettings(Y_CONTROLLER_SETTINGS);
        rotationalPidController = buildProfiledPIDControllerFromSettings(ROTATIONAL_CONTROLLER_SETTINGS);

        configureAutoBuilder();
        setupTargetPoseUpdateLoop();
        configureDashboard();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
                drivetrain::getEstimatedPose,
                drivetrain::resetPose,
                drivetrain::getSpeeds,
                this::driveByCorrectedSpeed,
                new PPHolonomicDriveController(
                        new PIDConstants(0, 0, 0),
                        new PIDConstants(0, 0, 0)),
                CONFIG,
                AutonomousContainer::shouldMirror,
                drivetrain
        );
    }

    private ChassisSpeeds getPIDChassisSpeedsToPose(Pose2d targetPose) {
        Pose2d currentPose = drivetrain.getEstimatedPoseByLatency(drivetrain.getSpeeds(), ROBOT_POSE_LATENCY);
        double xSpeed = xPidController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yPidController.calculate(currentPose.getY(), targetPose.getY());
        double rotationalSpeed = rotationalPidController.calculate(drivetrain.getAngle().getRadians(),
                targetPose.getRotation().getRadians());

        xSpeed = xPidController.atSetpoint() ? 0 : xSpeed;
        ySpeed = yPidController.atSetpoint() ? 0 : ySpeed;
        rotationalSpeed = rotationalPidController.atSetpoint() ? 0 : rotationalSpeed;

        return new ChassisSpeeds(xSpeed, ySpeed, rotationalSpeed);
    }

    private ChassisSpeeds getScaledFFSpeeds(ChassisSpeeds feedForwardSpeeds) {
        return feedForwardSpeeds.times(FF_SCALER);
    }

    private void driveByCorrectedSpeed(ChassisSpeeds feedForwardSpeeds) {
        ChassisSpeeds calculatePID = getPIDChassisSpeedsToPose(pathplannerTargetPose);
        ChassisSpeeds scaledFeedForward = getScaledFFSpeeds(feedForwardSpeeds);
        ChassisSpeeds output = calculatePID.plus(scaledFeedForward);
        drivetrain.drive(
                output.vxMetersPerSecond,
                output.vyMetersPerSecond,
                output.omegaRadiansPerSecond,
                false,
                TIME_STEP
        );
    }

    private void setupTargetPoseUpdateLoop() {
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> pathplannerTargetPose = pose);
    }

    private static PIDController buildPIDControllerFromSettings(PIDSettings settings) {
        return new PIDController(settings.getkP(), settings.getkI(), settings.getkD());
    }

    private static ProfiledPIDController buildProfiledPIDControllerFromSettings(PIDSettings settings) {
        return new ProfiledPIDController(settings.getkP(), settings.getkI(), settings.getkD(),
                new TrapezoidProfile.Constraints(2, 4));
    }

    public void configureDashboard() {
        namespace.putBoolean("should mirror", AutonomousContainer::shouldMirror);
    }

    public void updateNamespace() {
        namespace.update();
    }

    public static boolean shouldMirror() {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Blue).isPresent();
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException("path planner config missing");
        }
    }
}
