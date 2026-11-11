package frc.robot.swerve.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    @Override
    public void configureDashboard() {
        namespace.putData("swerve state", this);
        namespace.putData("field", field2d);
        namespace.putRunnable("reset gyro", this::resetAngleSensor);
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