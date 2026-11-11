package frc.robot.swerve.subsystem;

import com.ctre.phoenix6.CANBus;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;

public class SwerveConstants {

    interface MODULE {
        double MIN_SPEED_LIMIT = 0.1;
        double DRIVE_GEAR_RATIO = 1 / 6.12;
        double TURN_GEAR_RATIO = (double) 7 / 150;
        double WHEEL_DIAMETER_METERS = 0.1016;
        int DEGREES_IN_ROTATION = 360;
        int SECONDS_IN_MINUTE = 60;
        int ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;
    }

    interface DRIVETRAIN {
        String NAMESPACE_NAME = "swerve";
        CANBus CAN_BUS = new CANBus("canivore");
        double MAX_POSSIBLE_VELOCITY = 5.1;
        double TRACK_WIDTH = 0.55;
        double TRACK_LENGTH = 0.55;
        int GYRO_OFFSET = 180;
    }

    interface MODULE_HOLDER {
        String NAMESPACE_NAME = "swerve module holder";
        CANBus CAN_BUS = new CANBus("canivore");

        boolean FRONT_LEFT_DRIVE_INVERTED = true;
        boolean FRONT_RIGHT_DRIVE_INVERTED = false;
        boolean BACK_LEFT_DRIVE_INVERTED = false;
        boolean BACK_RIGHT_DRIVE_INVERTED = true;

        boolean FRONT_LEFT_CANCODER_INVERTED = false;
        boolean FRONT_RIGHT_CANCODER_INVERTED = false;
        boolean BACK_LEFT_CANCODER_INVERTED = false;
        boolean BACK_RIGHT_CANCODER_INVERTED = false;

        String FRONT_LEFT_NAMESPACE_NAME = "front left";
        String FRONT_RIGHT_NAMESPACE_NAME = "front right";
        String BACK_LEFT_NAMESPACE_NAME = "back left";
        String BACK_RIGHT_NAMESPACE_NAME = "back right";

        double FRONT_LEFT_OFFSET = -121.640625;
        double FRONT_RIGHT_OFFSET = 27.50976562;
        double BACK_LEFT_OFFSET = -18.10546875;
        double BACK_RIGHT_OFFSET = -12.04101563;

        PIDSettings drivePIDSettings =
                new PIDSettings(0.015, 0, 0, 0, 0, 0);

        PIDSettings turnPIDSettings =
                new PIDSettings(0.02, 0, 0.001, 0, 0, 0);

        FeedForwardSettings driveFeedForwardSettings = new FeedForwardSettings(
                0.014, 0.175, 0, FeedForwardController.ControlMode.LINEAR_VELOCITY);

        FeedForwardSettings turnFeedForwardSettings = new FeedForwardSettings(0, 0, 0,
                FeedForwardController.ControlMode.LINEAR_POSITION);

        boolean UPDATE_TURN_PERIODICALLY = false;
        boolean UPDATE_DRIVE_PERIODICALLY = false;
    }
}
