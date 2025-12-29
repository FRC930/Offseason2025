package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.EnumState;
import frc.robot.util.LoggedTunableGainsBuilder;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase implements ElevatorEvents {
    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged logged = new ElevatorInputsAutoLogged();
    private final EnumState<ElevatorState> state = new EnumState<>(ElevatorState.STOW);

    public static final double SPOOL_RADIUS = 1.751 / 2.0;
    public static final double INCHES_PER_ROT =
            (2.0 * (2.0 * Math.PI * SPOOL_RADIUS)); // Multiplied by 2 because its a 2
    // stage cascading elevator
    public static final double REDUCTION = (11.0 / 4.0);
    public static final double TOLERANCE = 1.0;

    public LoggedTunableGainsBuilder tunableGains = new LoggedTunableGainsBuilder(
            "Gains/Elevator/", 400.0, 0, 25.0, 0.0, 30.0, 0.0, 0.0, 75.0, 30.0, 0.0, 0.0, 0.0);

    public Elevator(ElevatorIO elevatorIO) {
        this.io = elevatorIO;
        io.setGains(tunableGains.build());
        RobotState.instance().setElevatorSource(logged.distance);
    }

    @Override
    public void periodic() {
        tunableGains.ifGainsHaveChanged((gains) -> this.io.setGains(gains));
        io.updateInputs(logged);
        Logger.processInputs("RobotState/Elevator", logged);

        switch (state.get()) {
            case STOW -> {}
            case L1 -> {}
            case L2 -> {}
            case L3 -> {}
            case L4 -> {}
            default -> {}
        }

        io.setTarget(Inches.of(state.get().getTargetPosition().getAsDouble()));
    }

    public Command setState(ElevatorState state) {
        return runOnce(() -> this.state.set(state));
    }

    @Override
    public Trigger atTarget() {
        return new Trigger(() -> {
            return MathUtil.isNear(
                    logged.distance.in(Inches), state.get().getTargetPosition().getAsDouble(), TOLERANCE);
        });
    }

    @Override
    public Trigger atStow() {
        return atTarget().and(state.is(ElevatorState.STOW));
    }

    @Override
    public Trigger atL1() {
        return atTarget().and(state.is(ElevatorState.L1));
    }

    @Override
    public Trigger atL2() {
        return atTarget().and(state.is(ElevatorState.L2));
    }

    @Override
    public Trigger atL3() {
        return atTarget().and(state.is(ElevatorState.L3));
    }

    @Override
    public Trigger atL4() {
        return atTarget().and(state.is(ElevatorState.L4));
    }
}
