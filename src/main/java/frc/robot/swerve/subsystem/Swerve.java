package frc.robot.swerve.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.RobotMap;

import static frc.robot.swerve.subsystem.SwerveConstants.DRIVETRAIN.*;

public class Swerve extends SwerveDrivetrain {

    private final Pigeon2 gyro;
    private final Field2d field2d;

    private final boolean isSimulated;

    private double simulatedYawDegrees;
    Pose2d simPose = new Pose2d();

    private static SwerveDrivePoseEstimator odometry;

    private static final double TRANSLATION_POSE_TOLERANCE = -1;
    private static final double TRANSLATION_VELOCITY_TOLERANCE = -1;
    private static final double ROTATION_TOLERANCE_IN_DEGREES = -1;
    private static final double ROTATION_VELOCITY_TOLERANCE = -1;

    private static Swerve instance;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve(NAMESPACE_NAME, SwerveModuleHolder.getFrontLeft(),
                    SwerveModuleHolder.getFrontRight(), SwerveModuleHolder.getBackLeft(),
                    SwerveModuleHolder.getBackRight(), TRACK_WIDTH, TRACK_LENGTH, MAX_POSSIBLE_VELOCITY,
                    new Pigeon2(RobotMap.CAN.SWERVE_GYRO_PIGEON_2_ID, CAN_BUS));
        }
        return instance;
    }

    private Swerve(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule,
                   SwerveModule backLeftModule, SwerveModule backRightModule,
                   double drivetrainTrackWidth, double drivetrainTrackLength,
                   double maxPossibleVelocity, Pigeon2 gyro) {
        super(namespaceName, frontLeftModule, frontRightModule, backLeftModule, backRightModule,
                drivetrainTrackWidth, drivetrainTrackLength, maxPossibleVelocity);
        this.gyro = gyro;
        field2d = new Field2d();
        isSimulated = RobotBase.isSimulation();
        odometry = new SwerveDrivePoseEstimator(getKinematics(), getAngle(), getSwerveModulePositions(), new Pose2d());
        configureDashboard();
    }

    @Override
    public Rotation2d getAngle() {
        if (isSimulated) {
            return Rotation2d.fromDegrees(simulatedYawDegrees);
        }
        return gyro.getRotation2d();
    }

    @Override
    public void resetAngleSensor() {
        setGyro(GYRO_OFFSET);
    }

    public void setGyro(double angle) {
        gyro.setYaw(angle);
        if (isSimulated) {
            simulatedYawDegrees = angle;
            simPose = new Pose2d(simPose.getTranslation(), Rotation2d.fromDegrees(angle));
        }
    }

    public Pose2d getEstimatedPose() {
        return odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        odometry.resetPose(newPose);
    }

    public void updateOdometry() {
        odometry.update(getAngle(), getSwerveModulePositions());
    }

    private boolean atRotation(Rotation2d rotation2d) {
        double error = rotation2d.minus(getAngle()).getDegrees();
        boolean isAtRotation = Math.abs(error) <= ROTATION_TOLERANCE_IN_DEGREES;
        boolean isRotationStill = Math.abs(getSpeeds().omegaRadiansPerSecond)
                <= ROTATION_VELOCITY_TOLERANCE;
        return isAtRotation && isRotationStill;
    }

    private boolean atAxis(double currentAxisPose, double targetAxisPose, double currentVelocity) {
        boolean atPose = MathUtil.isNear(targetAxisPose, currentAxisPose, TRANSLATION_POSE_TOLERANCE);
        boolean robotStill = Math.abs(currentVelocity) <= TRANSLATION_VELOCITY_TOLERANCE;
        return atPose && robotStill;
    }

    public boolean atPose(Pose2d pose2d) {
        boolean atXAxis = atAxis(getEstimatedPose().getX(), pose2d.getX(),
                getSpeeds().vxMetersPerSecond);
        boolean atYAxis = atAxis(getEstimatedPose().getY(), pose2d.getY(),
                getSpeeds().vyMetersPerSecond);
        return atXAxis && atYAxis && atRotation(pose2d.getRotation());
    }

    public Pose2d getEstimatedPoseByLatency(ChassisSpeeds relativeSpeeds, double latencySeconds) {
        double predictedXSpeed = relativeSpeeds.vxMetersPerSecond * latencySeconds;
        double predictedYSpeed = relativeSpeeds.vyMetersPerSecond * latencySeconds;
        Rotation2d predictedRotationSpeed =
                Rotation2d.fromRadians(relativeSpeeds.omegaRadiansPerSecond * latencySeconds);
        return getEstimatedPose().
                transformBy(new Transform2d(predictedXSpeed, predictedYSpeed, predictedRotationSpeed));
    }

    @Override
    public void configureDashboard() {
        namespace.putData("swerve state", this);
        namespace.putData("field", field2d);
        namespace.putRunnable("reset gyro", this::resetAngleSensor);
        namespace.putNumber("odometry x", () -> odometry.getEstimatedPosition().getX());
        namespace.putNumber("odometry y", () -> odometry.getEstimatedPosition().getY());
        namespace.putNumber("odometry rotation", () -> odometry.getEstimatedPosition().getRotation().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds speeds = getSpeeds();
        simPose = simPose.exp(new Twist2d(
                speeds.vxMetersPerSecond * DELTA_TIME,
                speeds.vyMetersPerSecond * DELTA_TIME,
                speeds.omegaRadiansPerSecond * DELTA_TIME)
        );

        simulatedYawDegrees = simPose.getRotation().getDegrees();
        field2d.setRobotPose(simPose);
    }
}