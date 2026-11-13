package com.spikes2212.util.smartmotorcontrollers;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.spikes2212.control.FFController;
import com.spikes2212.control.FFSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * A {@link SmartMotorController} representation of a {@link SparkBase} motor controller.
 *
 * @author Camellia Lami
 * @see SmartMotorController
 */
public class SparkWrapper implements SmartMotorController {

    /**
     * The feed forward controller running the feed forward calculations.
     */
    private final FFController ffController;

    /**
     * The spark motor controller.
     */
    private SparkBase sparkBase;

    /**
     * The spark's configuration accessor.
     */
    private SparkBaseConfigAccessor configAccessor;

    /**
     * The spark's configuration.
     */
    private SparkBaseConfig sparkConfig;

    /**
     * The closed loop configuration for the spark.
     */
    private ClosedLoopConfig closedLoopConfig;

    /**
     * The encoder configuration for the spark.
     */
    private EncoderConfig encoderConfig;

    private SparkSim sparkSim;
    private DCMotorSim motorSim;
    private double velocityConversionFactor;

    /**
     * Constructs a new instance of {@link SparkWrapper} with a {@link SparkMax} as the motor controller.
     *
     * @param deviceID    the spark's ID
     * @param type        the motor type
     * @param controlMode the type of feed forward control to use
     */
    public static SparkWrapper createSparkMax(int deviceID, SparkLowLevel.MotorType type,
                                              FFController.ControlMode controlMode) {
        SparkWrapper sparkWrapper = new SparkWrapper(controlMode);
        sparkWrapper.sparkBase = new SparkMax(deviceID, type);
        sparkWrapper.sparkConfig = new SparkMaxConfig();
        sparkWrapper.configAccessor = ((SparkMax) (sparkWrapper.sparkBase)).configAccessor;
        return sparkWrapper;
    }

    /**
     * Constructs a new instance of {@link SparkWrapper} with a {@link SparkMax} as the motor controller and with a
     * default {@link FFController.ControlMode} of linear velocity.
     *
     * @param deviceID the spark's ID
     * @param type     the motor type
     */
    public static SparkWrapper createSparkMax(int deviceID, SparkLowLevel.MotorType type) {
        return createSparkMax(deviceID, type, FFController.ControlMode.LINEAR_VELOCITY);
    }

    /**
     * Constructs a new instance of {@link SparkWrapper} with a {@link SparkFlex} as the motor controller.
     *
     * @param deviceID    the spark's ID
     * @param type        the motor type
     * @param controlMode the type of feed forward control to use
     */
    public static SparkWrapper createSparkFlex(int deviceID, SparkLowLevel.MotorType type,
                                               FFController.ControlMode controlMode) {
        SparkWrapper sparkWrapper = new SparkWrapper(controlMode);
        sparkWrapper.sparkBase = new SparkFlex(deviceID, type);
        sparkWrapper.sparkConfig = new SparkFlexConfig();
        sparkWrapper.configAccessor = ((SparkFlex) (sparkWrapper.sparkBase)).configAccessor;
        return sparkWrapper;
    }

    /**
     * Constructs a new instance of {@link SparkWrapper} with a {@link SparkFlex} as the motor controller and with a
     * default {@link FFController.ControlMode} of linear velocity.
     *
     * @param deviceID the spark's ID
     * @param type     the motor type
     */
    public static SparkWrapper createSparkFlex(int deviceID, SparkLowLevel.MotorType type) {
        return createSparkFlex(deviceID, type, FFController.ControlMode.LINEAR_VELOCITY);
    }

    private SparkWrapper(FFController.ControlMode controlMode) {
        ffController = new FFController(new FFSettings(controlMode));
        closedLoopConfig = new ClosedLoopConfig();
        encoderConfig = new EncoderConfig();
    }

    public SparkBaseConfig getSparkConfiguration() {
        return sparkConfig;
    }

    public void applyConfiguration(SparkBaseConfig newConfig) {
        sparkConfig.apply(newConfig);
        sparkBase.configure(sparkConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public ClosedLoopConfig getClosedLoopConfiguration() {
        return closedLoopConfig;
    }

    public void applyClosedLoopConfig(ClosedLoopConfig newConfig) {
        closedLoopConfig.apply(newConfig);
        sparkBase.configure(sparkConfig.apply(closedLoopConfig), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void restoreFactoryDefaults() {
        if (sparkBase instanceof SparkMax) sparkConfig = new SparkMaxConfig();
        else if (sparkBase instanceof SparkFlex) sparkConfig = new SparkFlexConfig();
        closedLoopConfig = new ClosedLoopConfig();
        encoderConfig = new EncoderConfig();
        sparkBase.configure(sparkConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void configureSimulation(DCMotor motorModel, double gearing, double moi) {
        sparkSim = new SparkSim(sparkBase, motorModel);
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(motorModel, moi, gearing),
                motorModel
        );
    }

    public void updateSimulation(double dt) {
        if (sparkSim == null || motorSim == null) {
            return;
        }
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setInputVoltage(sparkSim.getAppliedOutput() * batteryVoltage);
        motorSim.update(dt);
        double motorRpm = Units.radiansPerSecondToRotationsPerMinute(motorSim.getAngularVelocityRadPerSec());
        sparkSim.iterate(motorRpm * velocityConversionFactor, batteryVoltage, dt);
    }

    @Override
    public void set(double speed) {
        sparkBase.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        sparkBase.setVoltage(voltage);
    }

    @Override
    public double get() {
        return sparkBase.get();
    }

    @Override
    public void setInverted(boolean inverted) {
        int leaderID = configAccessor.getFollowerModeLeaderId();
        if (leaderID != 0) {
            sparkConfig.follow(leaderID, inverted);
        } else sparkConfig.inverted(inverted);
        sparkBase.configure(sparkConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public boolean getInverted() {
        return configAccessor.getInverted();
    }

    @Override
    public void disable() {
        sparkBase.disable();
    }

    @Override
    public void stopMotor() {
        sparkBase.stopMotor();
    }

    public void follow(SparkWrapper master) {
        setIdleMode(master.configAccessor.getIdleMode());
        sparkBase.configure(sparkConfig.follow(master.sparkBase, getInverted()),
                ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void unfollow() {
        sparkConfig.disableFollowerMode();
        sparkBase.configure(sparkConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void setIdleMode(SparkBaseConfig.IdleMode idleMode) {
        sparkConfig.idleMode(idleMode);
        sparkBase.configure(sparkConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void configurePID(PIDSettings pidSettings) {
        closedLoopConfig.pid(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        closedLoopConfig.iZone(pidSettings.getIZone());
        sparkBase.configure(sparkConfig.apply(closedLoopConfig), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void configureFF(FFSettings ffSettings) {
        ffController.setGains(ffSettings);
    }

    @Override
    public void configureTrapezoid(TrapezoidProfileSettings trapezoidProfileSettings) {
        MAXMotionConfig maxMotionConfig = new MAXMotionConfig();
        maxMotionConfig.cruiseVelocity(trapezoidProfileSettings.getMaxVelocity()).
                maxAcceleration(trapezoidProfileSettings.getMaxAcceleration());
        closedLoopConfig.apply(maxMotionConfig);
        // @TODO add s-curve when REV implements it
        sparkBase.configure(sparkConfig.apply(closedLoopConfig), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void setPosition(double position) {
        sparkBase.getEncoder().setPosition(position);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FFSettings ffSettings, TrapezoidProfileSettings trapezoidProfileSettings,
                       boolean updatePeriodically) {
        if (updatePeriodically) configureLoop(pidSettings, ffSettings, trapezoidProfileSettings);
        double source;
        if (ffController.getControlMode() == FFController.ControlMode.LINEAR_POSITION ||
                ffController.getControlMode() == FFController.ControlMode.ANGULAR_POSITION) {
            source = sparkBase.getEncoder().getPosition();
        } else {
            source = sparkBase.getEncoder().getVelocity();
        }
        sparkBase.getClosedLoopController().setReference(setpoint, controlMode.getSparkControlType(),
                ClosedLoopSlot.kSlot0, ffController.calculate(source, setpoint));
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, double acceleration, PIDSettings pidSettings,
                       FFSettings ffSettings, boolean updatePeriodically) {
        if (updatePeriodically) configureLoop(pidSettings, ffSettings,
                TrapezoidProfileSettings.EMPTY_TRAPEZOID_PROFILE_SETTINGS);
        double source;
        if (ffSettings.getControlMode() == FFController.ControlMode.LINEAR_POSITION ||
                ffSettings.getControlMode() == FFController.ControlMode.ANGULAR_POSITION) {
            source = sparkBase.getEncoder().getPosition();
        } else {
            source = sparkBase.getEncoder().getVelocity();
        }
            sparkBase.getClosedLoopController().setReference(setpoint, controlMode.getSparkControlType(),
                ClosedLoopSlot.kSlot0, ffController.calculate(source, setpoint, acceleration));
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FFSettings ffSettings, boolean updatePeriodically) {
        pidSet(controlMode, setpoint, 0, pidSettings, ffSettings, updatePeriodically);
    }

    @Override
    public boolean onTarget(UnifiedControlMode controlMode, double tolerance, double setpoint) {
        double value = switch (controlMode) {
            case PERCENT_OUTPUT -> sparkBase.getAppliedOutput();
            case VELOCITY -> getVelocity();
            case CURRENT -> getCurrent();
            case VOLTAGE -> getVoltage() * sparkBase.getAppliedOutput();
            default -> getPosition();
        };
        return Math.abs(value - setpoint) <= tolerance;
    }

    public void setPositionConversionFactor(double factor) {
        encoderConfig.positionConversionFactor(factor);
        sparkBase.configure(sparkConfig.apply(encoderConfig), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void setVelocityConversionFactor(double factor) {
        velocityConversionFactor = factor;
        encoderConfig.velocityConversionFactor(factor);
        sparkBase.configure(sparkConfig.apply(encoderConfig), ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public double getPosition() {
        return sparkBase.getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        return sparkBase.getEncoder().getVelocity();
    }

    public double getVoltage() {
        return sparkBase.getBusVoltage();
    }

    public double getCurrent() {
        return sparkBase.getOutputCurrent();
    }

    public SparkBase getDevice() {
        return sparkBase;
    }
}
