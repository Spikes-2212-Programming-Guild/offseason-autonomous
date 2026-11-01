package frc.robot;

public class RobotMap {

    public interface CAN {

        int SWERVE_FRONT_LEFT_DRIVE_TALON_FX_ID = 1;
        int SWERVE_FRONT_RIGHT_DRIVE_TALON_FX_ID = 4;
        int SWERVE_BACK_LEFT_DRIVE_TALON_FX_ID = 7;
        int SWERVE_BACK_RIGHT_DRIVE_TALON_FX_ID = 10;

        int SWERVE_FRONT_LEFT_TURN_SPARK_MAX_ID = 41;
        int SWERVE_FRONT_RIGHT_TURN_SPARK_MAX_ID = 5;
        int SWERVE_BACK_LEFT_TURN_SPARK_MAX_ID = 8;
        int SWERVE_BACK_RIGHT_TURN_SPARK_MAX_ID = 11;

        int SWERVE_FRONT_LEFT_ABSOLUTE_ENCODER_ID = 3;
        int SWERVE_FRONT_RIGHT_ABSOLUTE_ENCODER_ID = 6;
        int SWERVE_BACK_LEFT_ABSOLUTE_ENCODER_ID = 9;
        int SWERVE_BACK_RIGHT_ABSOLUTE_ENCODER_ID = 12;

        int SWERVE_GYRO_PIGEON_2_ID = 13;
    }

    public interface DIO {

    }

    public interface PWM {

    }

    public interface AIN {

    }

    public interface PCM {

    }
}
