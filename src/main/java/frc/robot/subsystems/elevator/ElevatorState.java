package frc.robot.subsystems.elevator;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public enum ElevatorState {
    STOW(new LoggedTunableNumber("Elevator/Levels/L0", 0.0)),
    L1(new LoggedTunableNumber("Elevator/Levels/L1", 12.0)),
    L2(new LoggedTunableNumber("Elevator/Levels/L2", 19.8)),
    L3(new LoggedTunableNumber("Elevator/Levels/L3", 35.3)),
    L4(new LoggedTunableNumber("Elevator/Levels/L4", 57.0));

    private final DoubleSupplier targetPosition;

    ElevatorState(DoubleSupplier position) {
        this.targetPosition = position;
    }

    public DoubleSupplier getTargetPosition() {
        return targetPosition;
    }
}
