package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.swerve.subsystem.Drivetrain;

public class OI /*GEVALD*/ {

    private final PlaystationControllerWrapper controller;

    public OI() {
        controller = new PlaystationControllerWrapper(0);

        controller.getCircleButton().onTrue(new InstantCommand(() -> Drivetrain.getInstance().resetFieldRelativity()));
    }

    public double getLeftX() {
        return controller.getLeftX();
    }

    public double getLeftY() {
        return controller.getLeftY();
    }

    public double getRightX() {
        return controller.getRightX();
    }
}
