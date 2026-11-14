package frc.robot.swerve.commands;

import com.spikes2212.commandbased.swerve.commands.DriveSwerve;
import frc.robot.swerve.subsystem.Swerve;

import java.util.function.Supplier;

public class Drive extends DriveSwerve {

    public Drive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double>
            rotationSpeed, boolean isFieldRelative) {
        super(Swerve.getInstance(), xSpeed, ySpeed, rotationSpeed, isFieldRelative);
    }
}
