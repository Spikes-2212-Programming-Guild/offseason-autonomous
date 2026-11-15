// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.swerve.subsystem.Swerve;
import frc.robot.swerve.subsystem.SwerveModuleHolder;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    private final RootNamespace namespace;
    private final Swerve swerve;
    private final Supplier<Double> test;
//    AutonomousContainer container = new AutonomousContainer(swerve);

    public Robot() {
        CommandScheduler.getInstance().cancelAll();
        namespace = new RootNamespace("robot");
        test = namespace.addTunableDouble("test", 123);
        swerve = Swerve.getInstance();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SwerveModuleHolder.updateNamespace();
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
//        namespace.publishDouble("test", () -> 5.0);
        swerve.setDefaultCommand(swerve.driveWithController(
                () -> deadband(oi.getLeftX()),
                () -> deadband(oi.getLeftY()),
                () -> deadband(oi.getRightX()), true, 5.1));
    }

    private static double deadband(double input) {
        if (Math.abs(input) < 0.05) return 0;
        return MathUtil.copyDirectionPow(input, 2);
    }

    @Override
    public void teleopPeriodic() {
        DataLogManager.log(String.valueOf(test.get()));
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
