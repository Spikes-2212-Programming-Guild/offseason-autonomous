package frc.robot.swerve.subsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class SwerveModuleHolder {

    private static final RootNamespace namespace = new RootNamespace("swerve module holder");

    private final static CANBus CANIVORE = new CANBus("canivore");

    private static final boolean FRONT_LEFT_DRIVE_INVERTED = true;
    private static final boolean FRONT_RIGHT_DRIVE_INVERTED = false;
    private static final boolean BACK_LEFT_DRIVE_INVERTED = false;
    private static final boolean BACK_RIGHT_DRIVE_INVERTED = true;

    private static final boolean FRONT_LEFT_CANCODER_INVERTED = false;
    private static final boolean FRONT_RIGHT_CANCODER_INVERTED = false;
    private static final boolean BACK_LEFT_CANCODER_INVERTED = false;
    private static final boolean BACK_RIGHT_CANCODER_INVERTED = false;

    private static final String FRONT_LEFT_NAMESPACE_NAME = "front left";
    private static final String FRONT_RIGHT_NAMESPACE_NAME = "front right";
    private static final String BACK_LEFT_NAMESPACE_NAME = "back left";
    private static final String BACK_RIGHT_NAMESPACE_NAME = "back right";

    private static final double FRONT_LEFT_OFFSET = -121.640625;
    private static final double FRONT_RIGHT_OFFSET = 27.50976562;
    private static final double BACK_LEFT_OFFSET = -18.10546875;
    private static final double BACK_RIGHT_OFFSET = -12.04101563;

    private static final PIDSettings drivePIDSettings =
            new PIDSettings(0.015, 0, 0, 0, 0, 0);

    private static final PIDSettings turnPIDSettings =
            new PIDSettings(0.02, 0, 0.001, 0, 0, 0);

    private static final FeedForwardSettings driveFeedForwardSettings =
            new FeedForwardSettings(0.014, 0.175, 0, FeedForwardController.ControlMode.LINEAR_VELOCITY);

    private static final FeedForwardSettings turnFeedForwardSettings =
            new FeedForwardSettings(0, 0, 0, FeedForwardController.ControlMode.LINEAR_POSITION);

    public static final boolean UPDATE_TURN_PERIODICALLY = false;
    public static final boolean UPDATE_DRIVE_PERIODICALLY = false;

    private static SwerveModuleRebuilt frontLeft;
    private static SwerveModuleRebuilt frontRight;
    private static SwerveModuleRebuilt backLeft;
    private static SwerveModuleRebuilt backRight;

    public static SwerveModuleRebuilt getFrontLeft() {
        if (frontLeft == null) {
            frontLeft = new SwerveModuleRebuilt(FRONT_LEFT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_FRONT_LEFT_DRIVE_TALON_FX_ID, CANIVORE),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_FRONT_LEFT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), FRONT_LEFT_DRIVE_INVERTED,
                    FRONT_LEFT_CANCODER_INVERTED, FRONT_LEFT_OFFSET, drivePIDSettings, turnPIDSettings,
                    driveFeedForwardSettings, turnFeedForwardSettings,
                    new CANcoder(RobotMap.CAN.SWERVE_FRONT_LEFT_ABSOLUTE_ENCODER_ID, CANIVORE));
        }
        return frontLeft;
    }

    public static SwerveModuleRebuilt getFrontRight() {
        if (frontRight == null) {
            frontRight = new SwerveModuleRebuilt(FRONT_RIGHT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_FRONT_RIGHT_DRIVE_TALON_FX_ID, CANIVORE),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_FRONT_RIGHT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), FRONT_RIGHT_DRIVE_INVERTED,
                    FRONT_RIGHT_CANCODER_INVERTED, FRONT_RIGHT_OFFSET, drivePIDSettings, turnPIDSettings,
                    driveFeedForwardSettings, turnFeedForwardSettings,
                    new CANcoder(RobotMap.CAN.SWERVE_FRONT_RIGHT_ABSOLUTE_ENCODER_ID, CANIVORE));
        }
        return frontRight;
    }

    public static SwerveModuleRebuilt getBackLeft() {
        if (backLeft == null) {
            backLeft = new SwerveModuleRebuilt(BACK_LEFT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_BACK_LEFT_DRIVE_TALON_FX_ID, CANIVORE),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_BACK_LEFT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), BACK_LEFT_DRIVE_INVERTED,
                    BACK_LEFT_CANCODER_INVERTED, BACK_LEFT_OFFSET, drivePIDSettings, turnPIDSettings,
                    driveFeedForwardSettings, turnFeedForwardSettings,
                    new CANcoder(RobotMap.CAN.SWERVE_BACK_LEFT_ABSOLUTE_ENCODER_ID, CANIVORE));
        }
        return backLeft;
    }

    public static SwerveModuleRebuilt getBackRight() {
        if (backRight == null) {
            backRight = new SwerveModuleRebuilt(BACK_RIGHT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_BACK_RIGHT_DRIVE_TALON_FX_ID, CANIVORE),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_BACK_RIGHT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), BACK_RIGHT_DRIVE_INVERTED,
                    BACK_RIGHT_CANCODER_INVERTED, BACK_RIGHT_OFFSET, drivePIDSettings, turnPIDSettings,
                    driveFeedForwardSettings, turnFeedForwardSettings,
                    new CANcoder(RobotMap.CAN.SWERVE_BACK_RIGHT_ABSOLUTE_ENCODER_ID, CANIVORE));
        }
        return backRight;
    }

    public static void updateNamespace() {
        namespace.update();
    }
}
