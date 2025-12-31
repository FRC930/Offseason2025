package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interface exposing only the triggers that behaviors might need from OperatorIntent.
 * Keeps the contract minimal - behaviors shouldn't know about raw button inputs.
 */
public interface OperatorIntentEvents {
    public Trigger wantsToScore();

    public Trigger wantsToIntake();

    public Trigger wantsToEject();

    // ==================== LEVEL SELECTION BUTTONS ====================

    public Trigger selectL1();

    public Trigger selectL2();

    public Trigger selectL3();

    public Trigger selectL4();
}
