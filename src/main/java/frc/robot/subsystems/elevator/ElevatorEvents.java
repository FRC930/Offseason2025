package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ElevatorEvents {
    public Trigger atTarget();

    public Trigger atStow();

    public Trigger atL1();

    public Trigger atL2();

    public Trigger atL3();

    public Trigger atL4();
}
