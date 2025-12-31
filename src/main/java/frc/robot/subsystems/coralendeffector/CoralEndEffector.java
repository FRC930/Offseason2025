package frc.robot.subsystems.coralendeffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import org.littletonrobotics.junction.Logger;

/**
 * <h1>Coral end effector</h1>
 * <p>
 * Controls the rollers on the end of our coral end effector
 * </p>
 * <ul>
 * <li>Voltage control</li>
 * </ul>
 */
public class CoralEndEffector extends SubsystemBase implements CoralEndEffectorEvents {

    public static final double CORAL_DISTANCE_THRESHOLD = 4.5;
    public static final double CORAL_STRENGTH_THRESHOLD = 8000.0;

    private final CoralEndEffectorIO io;
    private final CoralEndEffectorInputsAutoLogged logged = new CoralEndEffectorInputsAutoLogged();
    private final EnumState<CoralEndEffectorState> state = new EnumState<>(CoralEndEffectorState.IDLE);
    private final Debouncer hasCoralDebouncer = new Debouncer(1.0);

    public CoralEndEffector(CoralEndEffectorIO io) {
        this.io = io;
    }

    public void setTarget(Voltage target) {
        io.setTarget(target);
    }

    @Override
    public void periodic() {
        io.updateInputs(logged);

        Logger.processInputs("RobotState/CoralEndEffector", logged);
        Logger.recordOutput(
                "RobotState/CoralEndEffector/CurrentState", state.get().getLabel());

        switch (state.get()) {
            case INTAKING -> {
                if (hasCoralDebouncer.calculate(logged.hasCoral)) {
                    state.set(CoralEndEffectorState.IDLE);
                }
            }
            default -> {}
        }

        setTarget(state.get().getVoltage());
    }

    public Command intake() {
        return startEnd(
                () -> this.state.set(CoralEndEffectorState.INTAKING), () -> this.state.set(CoralEndEffectorState.IDLE));
    }

    public Command score() {
        return startEnd(
                () -> this.state.set(CoralEndEffectorState.SCORING), () -> this.state.set(CoralEndEffectorState.IDLE));
    }

    @Override
    public Trigger hasCoral() {
        return new Trigger(() -> logged.hasCoral);
    }

    public Command eject() {
        return startEnd(
                () -> this.state.set(CoralEndEffectorState.EJECTING), () -> this.state.set(CoralEndEffectorState.IDLE));
    }

    public Command idle() {
        return new InstantCommand(() -> this.state.set(CoralEndEffectorState.IDLE));
    }
}
