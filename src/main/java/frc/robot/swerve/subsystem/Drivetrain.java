package frc.robot.swerve.subsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotMap;

public class Drivetrain extends SwerveDrivetrain {

    public static final double MAX_POSSIBLE_VELOCITY = 5.1;

    private static final String NAMESPACE_NAME = "swerve drivetrain";

    private static final CANBus CANIVORE = new CANBus("canivore");

    private static final double TRACK_WIDTH = 0.55;
    private static final double TRACK_LENGTH = 0.55;
    private static final int GYRO_OFFSET = 180;

    private final Pigeon2 gyro;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(NAMESPACE_NAME, SwerveModuleHolder.getFrontLeft(),
                    SwerveModuleHolder.getFrontRight(), SwerveModuleHolder.getBackLeft(),
                    SwerveModuleHolder.getBackRight(), TRACK_WIDTH, TRACK_LENGTH, MAX_POSSIBLE_VELOCITY,
                    new Pigeon2(RobotMap.CAN.SWERVE_GYRO_PIGEON_2_ID, CANIVORE));
        }
        return instance;
    }

    private Drivetrain(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule,
                       SwerveModule backLeftModule, SwerveModule backRightModule,
                       double drivetrainTrackWidth, double drivetrainTrackLength,
                       double maxPossibleVelocity, Pigeon2 gyro) {
        super(namespaceName, frontLeftModule, frontRightModule, backLeftModule, backRightModule,
                drivetrainTrackWidth, drivetrainTrackLength, maxPossibleVelocity);
        this.gyro = gyro;
        configureDashboard();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    @Override
    public void resetAngleSensor() {
        gyro.setYaw(GYRO_OFFSET);
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative,
                      double timeStep, boolean useVelocityPID) {
        super.drive(xSpeed, ySpeed, rotationSpeed, isFieldRelative,
                timeStep, useVelocityPID);
    }

    public void setGyro(double angle) {
        gyro.setYaw(angle);
    }

    @Override
    public void configureDashboard() {
        namespace.putData("swerve", this);
        namespace.putRunnable("reset gyro", () -> gyro.setYaw(0));
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
