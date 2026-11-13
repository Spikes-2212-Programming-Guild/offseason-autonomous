package frc.robot;

import com.spikes2212.util.XboxControllerWrapper;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.swerve.subsystem.Swerve;

public class OI /*GEVALD*/ {

    private final XboxControllerWrapper controller;

    public OI() {
        controller = new XboxControllerWrapper(0);

        controller.getBButton().onTrue(new InstantCommand(() -> Swerve.getInstance().resetFieldRelativity()));
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
