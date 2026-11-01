package frc.robot.swerve.subsystem;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

public class SwerveModuleRebuilt extends SwerveModule {

    private static final double MIN_SPEED_LIMIT = 0.1;
    private static final double DRIVE_GEAR_RATIO = 1 / 6.12;
    private static final double TURN_GEAR_RATIO = (double) 7 / 150;
    private static final double WHEEL_DIAMETER_METERS = 0.1016;
    private static final int DEGREES_IN_ROTATION = 360;
    private static final int SECONDS_IN_MINUTE = 60;
    private static final int ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;

    private static final double DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS =
            DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI;
    private static final double TURN_VELOCITY_IN_ROTATION =
            (TURN_GEAR_RATIO * DEGREES_IN_ROTATION) / SECONDS_IN_MINUTE;
    private static final double TURN_POSITION_IN_ROTATION =
            TURN_GEAR_RATIO * DEGREES_IN_ROTATION;

    private final TalonFXWrapper driveMotor;
    private final SparkWrapper turnMotor;
    private final CANcoder cancoder;

    public SwerveModuleRebuilt(String namespaceName, TalonFXWrapper driveMotor, SparkWrapper turnMotor,
                               boolean driveMotorInverted, boolean turnMotorInverted,
                               double absoluteEncoderOffset, PIDSettings driveMotorPIDSettings,
                               PIDSettings turnMotorPIDSettings, FeedForwardSettings driveMotorFeedForwardSettings,
                               FeedForwardSettings turnMotorFeedForwardSettings, CANcoder cancoder) {
        super(namespaceName, driveMotor, turnMotor, driveMotorInverted, turnMotorInverted, absoluteEncoderOffset,
                driveMotorPIDSettings, turnMotorPIDSettings, driveMotorFeedForwardSettings,
                turnMotorFeedForwardSettings, MIN_SPEED_LIMIT);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.cancoder = cancoder;
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorInverted);
        turnMotor.setInverted(!turnMotorInverted);

        configureTurnController();
        configureDriveController();
        configureAbsoluteEncoder();
        configureDashboard();
    }

    @Override
    protected void configureDriveController() {
        driveMotor.setIdleMode(NeutralModeValue.Brake);
        driveMotor.setEncoderConversionFactor(DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS);
        driveMotor.configureLoop(driveMotorPIDSettings, driveMotorFeedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureTurnController() {
        turnMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        turnMotor.setPositionConversionFactor(TURN_POSITION_IN_ROTATION);
        turnMotor.setVelocityConversionFactor(TURN_VELOCITY_IN_ROTATION);
        turnMotor.configureLoop(turnMotorPIDSettings, turnMotorFeedForwardSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs().
                withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION_DISCONTINUITY_POINT).
                withSensorDirection(turnMotorInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(absoluteEncoderOffset / 360.0);
        cancoder.getConfigurator().apply(magnetSensorConfigs);
    }

    @Override
    protected Rotation2d getAbsoluteModuleAngle() {
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue().in(Units.Degree));
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute encoder", () -> this.getAbsoluteModuleAngle().getDegrees());
        namespace.putNumber("relative angle", this::getRelativeModuleAngle);
//
//        Supplier<Double> targetAngle = namespace.addConstantDouble("target angle", 0);
//        namespace.putCommand("set target angle", Commands.runEnd(
//                () -> setTargetAngle(Rotation2d.fromDegrees(targetAngle.get())),
//                this::stop,
//                this
//        ));

//        namespace.putNumber("velocity", this::getDrivingVelocity);
//        Supplier<Double> targetVelocity = namespace.addConstantDouble("target velocity", 0);
//        namespace.putCommand("set target velocity", Commands.runEnd(
//                () -> setTargetVelocity(targetVelocity.get(), 5.1, true),
//                this::stop,
//                this
//        ));
    }
}
