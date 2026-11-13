package frc.robot.swerve.subsystem;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

import static frc.robot.swerve.subsystem.SwerveConstants.MODULE.*;

public class SwerveModuleRebuilt extends SwerveModule {

    private static final double DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS =
            DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI;
    private static final double TURN_VELOCITY_IN_ROTATION =
            (TURN_GEAR_RATIO * DEGREES_IN_ROTATION) / SECONDS_IN_MINUTE;
    private static final double TURN_POSITION_IN_ROTATION =
            TURN_GEAR_RATIO * DEGREES_IN_ROTATION;

    private final TalonFXWrapper driveMotor;
    private final SparkWrapper turnMotor;
    private final CANcoder cancoder;

    private DCMotorSim driveMotorSim;
    private TalonFXSimState talonFXSim;
    private CANcoderSimState cancoderSim;

    public SwerveModuleRebuilt(String namespaceName, TalonFXWrapper driveMotor, SparkWrapper turnMotor,
                               boolean driveMotorInverted, boolean turnMotorInverted,
                               double absoluteEncoderOffset, CANcoder cancoder) {
        super(namespaceName, driveMotor, turnMotor, driveMotorInverted, turnMotorInverted, absoluteEncoderOffset,
                DRIVE_PID_SETTINGS, TURN_PID_SETTINGS, DRIVE_FF_SETTINGS, TURN_FF_SETTINGS, MIN_SPEED_LIMIT,
                UPDATE_DRIVE_PERIODICALLY, UPDATE_TURN_PERIODICALLY);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.cancoder = cancoder;
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorInverted);
        turnMotor.setInverted(!turnMotorInverted);

        if (RobotBase.isSimulation()) {
            this.absoluteEncoderOffset = 0;
            driveMotorPIDSettings = SIM_DRIVE_PID_SETTINGS;
            turnMotorPIDSettings = SIM_TURN_PID_SETTINGS;
            driveMotorFFSettings = SIM_DRIVE_FF_SETTINGS;
            turnMotorFFSettings = SIM_TURN_FF_SETTINGS;

            turnMotor.configureSimulation(DCMotor.getNEO(1), 1.0, 0.00001873);
            cancoderSim = cancoder.getSimState();
            talonFXSim = driveMotor.getSim();
            driveMotorSim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            DCMotor.getKrakenX60(1),
                            0.025,
                            1 / DRIVE_GEAR_RATIO
                    ),
                    DCMotor.getKrakenX60(1)
            );
        }

        configureTurnController();
        configureDriveController();
        configureAbsoluteEncoder();
        configureDashboard();
    }

    @Override
    protected void configureDriveController() {
        driveMotor.setIdleMode(NeutralModeValue.Brake);
        driveMotor.setEncoderConversionFactor(DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS);
        driveMotor.configureLoop(driveMotorPIDSettings, driveMotorFFSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureTurnController() {
        turnMotor.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        turnMotor.setPositionConversionFactor(TURN_POSITION_IN_ROTATION);
        turnMotor.setVelocityConversionFactor(TURN_VELOCITY_IN_ROTATION);
        turnMotor.configureLoop(turnMotorPIDSettings, turnMotorFFSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
    }

    @Override
    protected void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs().
                withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION_DISCONTINUITY_POINT).
                withSensorDirection(turnMotorInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(absoluteEncoderOffset / DEGREES_IN_ROTATION);
        cancoder.getConfigurator().apply(magnetSensorConfigs);
    }

    @Override
    protected Rotation2d getAbsoluteModuleAngle() {
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue().in(Units.Degree));
    }

    @Override
    public void simulationPeriodic() {
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        driveMotorSim.setInputVoltage(talonFXSim.getMotorVoltage());

        driveMotorSim.update(SwerveConstants.DRIVETRAIN.DELTA_TIME);

        double rotorPosition = driveMotorSim.getAngularPositionRotations() * 1 / DRIVE_GEAR_RATIO;
        double rotorVelocity = driveMotorSim.getAngularVelocityRPM() / SECONDS_IN_MINUTE * 1 / DRIVE_GEAR_RATIO;

        talonFXSim.setRawRotorPosition(rotorPosition);
        talonFXSim.setRotorVelocity(rotorVelocity);

        turnMotor.updateSimulation(SwerveConstants.DRIVETRAIN.DELTA_TIME);
        double moduleAngleDegrees = turnMotor.getPosition();
        double moduleRotations = moduleAngleDegrees / DEGREES_IN_ROTATION;
        double offsetRotations = absoluteEncoderOffset / DEGREES_IN_ROTATION;
        double rawPosition = moduleRotations - offsetRotations;
        cancoderSim.setRawPosition(rawPosition);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute encoder", () -> this.getAbsoluteModuleAngle().getDegrees());
        namespace.putNumber("relative angle", this::getRelativeModuleAngle);

        Supplier<Double> targetAngle = namespace.addConstantDouble("target angle", 0);
        namespace.putCommand("set target angle", Commands.runEnd(
                () -> setTargetAngle(Rotation2d.fromDegrees(targetAngle.get())),
                this::stop,
                this
        ));

        namespace.putNumber("velocity", this::getDrivingVelocity);
        Supplier<Double> targetVelocity = namespace.addConstantDouble("target velocity", 0);
        namespace.putCommand("set target velocity", Commands.runEnd(
                () -> setTargetVelocity(targetVelocity.get()),
                this::stop,
                this
        ));
    }
}
