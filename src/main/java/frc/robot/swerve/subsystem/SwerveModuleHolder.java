package frc.robot.swerve.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

import static frc.robot.swerve.subsystem.SwerveConstants.MODULE_HOLDER.*;

public class SwerveModuleHolder {

    private static final RootNamespace namespace = new RootNamespace(NAMESPACE_NAME);

    private static SwerveModuleRebuilt frontLeft;
    private static SwerveModuleRebuilt frontRight;
    private static SwerveModuleRebuilt backLeft;
    private static SwerveModuleRebuilt backRight;

    public static SwerveModuleRebuilt getFrontLeft() {
        if (frontLeft == null) {
            frontLeft = new SwerveModuleRebuilt(FRONT_LEFT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_FRONT_LEFT_DRIVE_TALON_FX_ID, CAN_BUS),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_FRONT_LEFT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), FRONT_LEFT_DRIVE_INVERTED,
                    FRONT_LEFT_CANCODER_INVERTED, FRONT_LEFT_OFFSET,
                    new CANcoder(RobotMap.CAN.SWERVE_FRONT_LEFT_ABSOLUTE_ENCODER_ID, CAN_BUS));
        }
        return frontLeft;
    }

    public static SwerveModuleRebuilt getFrontRight() {
        if (frontRight == null) {
            frontRight = new SwerveModuleRebuilt(FRONT_RIGHT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_FRONT_RIGHT_DRIVE_TALON_FX_ID, CAN_BUS),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_FRONT_RIGHT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), FRONT_RIGHT_DRIVE_INVERTED,
                    FRONT_RIGHT_CANCODER_INVERTED, FRONT_RIGHT_OFFSET,
                    new CANcoder(RobotMap.CAN.SWERVE_FRONT_RIGHT_ABSOLUTE_ENCODER_ID, CAN_BUS));
        }
        return frontRight;
    }

    public static SwerveModuleRebuilt getBackLeft() {
        if (backLeft == null) {
            backLeft = new SwerveModuleRebuilt(BACK_LEFT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_BACK_LEFT_DRIVE_TALON_FX_ID, CAN_BUS),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_BACK_LEFT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), BACK_LEFT_DRIVE_INVERTED,
                    BACK_LEFT_CANCODER_INVERTED, BACK_LEFT_OFFSET,
                    new CANcoder(RobotMap.CAN.SWERVE_BACK_LEFT_ABSOLUTE_ENCODER_ID, CAN_BUS));
        }
        return backLeft;
    }

    public static SwerveModuleRebuilt getBackRight() {
        if (backRight == null) {
            backRight = new SwerveModuleRebuilt(BACK_RIGHT_NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SWERVE_BACK_RIGHT_DRIVE_TALON_FX_ID, CAN_BUS),
                    SparkWrapper.createSparkMax(RobotMap.CAN.SWERVE_BACK_RIGHT_TURN_SPARK_MAX_ID,
                            SparkLowLevel.MotorType.kBrushless), BACK_RIGHT_DRIVE_INVERTED,
                    BACK_RIGHT_CANCODER_INVERTED, BACK_RIGHT_OFFSET,
                    new CANcoder(RobotMap.CAN.SWERVE_BACK_RIGHT_ABSOLUTE_ENCODER_ID, CAN_BUS));
        }
        return backRight;
    }

    public static void updateNamespace() {
        namespace.update();
    }
}
