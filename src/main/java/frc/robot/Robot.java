// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.swerve.commands.Drive;
import frc.robot.swerve.subsystem.Swerve;
import frc.robot.swerve.subsystem.SwerveModuleHolder;

public class Robot extends TimedRobot {

    private Swerve swerve = Swerve.getInstance();

    RootNamespace namespace = new RootNamespace("robot");
    AutonomousContainer container = new AutonomousContainer(swerve);


    @Override
    public void robotInit() {
        CommandScheduler.getInstance().cancelAll();
        Command auto = new PathPlannerAuto("New Auto");
        namespace.putCommand("auto", auto);
//        swerve = Swerve.getInstance();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SwerveModuleHolder.updateNamespace();
        swerve.updateOdometry();
        container.updateNamespace();
        namespace.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        swerve.resetFieldRelativity();
        swerve.resetRelativeEncoders();

        OI oi = new OI();
        double ySpeed = 2;
        double xSpeed = 2;
        double rotationSpeed = 2;
        swerve.setDefaultCommand(new Drive(
                () -> deadband(oi.getLeftY()) * xSpeed,
                () -> deadband(oi.getLeftX()) * ySpeed,
                () -> deadband(oi.getRightX()) * rotationSpeed, true, true));
    }

    private static double deadband(double input) {
        if (Math.abs(input) < 0.05) return 0;
        return MathUtil.copyDirectionPow(input, 2);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
