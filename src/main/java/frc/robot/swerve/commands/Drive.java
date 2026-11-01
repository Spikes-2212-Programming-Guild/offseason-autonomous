package frc.robot.swerve.commands;

import com.spikes2212.command.drivetrains.swerve.commands.DriveSwerve;
import frc.robot.swerve.subsystem.Drivetrain;

import java.util.function.Supplier;

public class Drive extends DriveSwerve {

    public Drive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double>
            rotationSpeed, boolean isFieldRelative, boolean useVelocityPID) {
        super(Drivetrain.getInstance(), xSpeed, ySpeed, rotationSpeed, isFieldRelative, useVelocityPID);
    }
}
