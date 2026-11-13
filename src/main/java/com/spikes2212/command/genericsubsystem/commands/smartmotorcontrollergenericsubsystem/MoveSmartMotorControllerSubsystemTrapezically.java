package com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FFSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.util.UnifiedControlMode;

import java.util.function.Supplier;


/**
 * A command that moves a {@link SmartMotorControllerGenericSubsystem} by running a trapezoid profile on its
 * master's control loops.
 *
 * @author Yoel Perman Brilliant
 * @see SmartMotorControllerGenericSubsystem
 * @see MoveSmartMotorControllerGenericSubsystem
 */
public class MoveSmartMotorControllerSubsystemTrapezically extends MoveSmartMotorControllerGenericSubsystem {

    /**
     * The loops' trapezoid profile configurations.
     */
    protected final TrapezoidProfileSettings trapezoidProfileSettings;

    /**
     * Constructs a new instance of {@link MoveSmartMotorControllerSubsystemTrapezically}.
     *
     * @param subsystem                the {@link SmartMotorControllerGenericSubsystem} this command will run on
     * @param pidSettings              the loop's PID constants
     * @param ffSettings      the loop's feed forward gains
     * @param setpoint                 the setpoint this command should bring the {@link SmartMotorControllerGenericSubsystem} to
     * @param trapezoidProfileSettings the trapezoid profile settings
     */
    public MoveSmartMotorControllerSubsystemTrapezically(SmartMotorControllerGenericSubsystem subsystem, PIDSettings pidSettings,
                                                         FFSettings ffSettings,
                                                         Supplier<Double> setpoint,
                                                         TrapezoidProfileSettings trapezoidProfileSettings,
                                                         boolean updatePeriodically) {
        super(subsystem, pidSettings, ffSettings, UnifiedControlMode.TRAPEZOID_PROFILE, setpoint, updatePeriodically);
        this.trapezoidProfileSettings = trapezoidProfileSettings;
    }

    @Override
    public void initialize() {
        subsystem.configureLoop(pidSettings, ffSettings, trapezoidProfileSettings);
    }

    @Override
    public void execute() {
        subsystem.pidSet(controlMode, setpoint.get(), pidSettings, ffSettings,
                trapezoidProfileSettings, updatePeriodically);
    }
}
