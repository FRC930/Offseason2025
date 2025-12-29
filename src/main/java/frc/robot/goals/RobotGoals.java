package frc.robot.goals;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.operator.OperatorIntent;
import frc.robot.operator.ScoringLevel;
import frc.robot.util.EnumState;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks current robot-level goals and composes intent + parameters.
 * This is the "API layer" - a pure state holder for robot goals.
 *
 * <p>RobotGoals does NOT wire its own transitions. That's the job of
 * {@link RobotGoalsBehavior}, which follows the same pattern as subsystem behaviors.
 *
 * <p>Responsibilities:
 * <ul>
 *   <li>Maintain current goal state</li>
 *   <li>Compose intent + selections into specific goals (scoringAtL1, etc.)</li>
 *   <li>Expose goal triggers for subsystem behaviors to react to</li>
 * </ul>
 */
public class RobotGoals extends VirtualSubsystem implements RobotGoalEvents {

    private final EnumState<RobotGoal> currentGoal = new EnumState<>("RobotGoals/Goal", RobotGoal.IDLE);
    private final OperatorIntent intent;

    public RobotGoals(OperatorIntent intent) {
        this.intent = intent;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "RobotGoals/SelectedLevel", intent.getSelectedLevel().name());
    }

    // ==================== GOAL MUTATION ====================

    public Command setGoal(RobotGoal goal) {
        return Commands.runOnce(() -> currentGoal.set(goal));
    }

    // ==================== BASIC GOAL TRIGGERS ====================

    @Override
    public Trigger isIdle() {
        return currentGoal.is(RobotGoal.IDLE);
    }

    @Override
    public Trigger isIntaking() {
        return currentGoal.is(RobotGoal.INTAKING);
    }

    @Override
    public Trigger isScoring() {
        return currentGoal.is(RobotGoal.SCORING);
    }

    @Override
    public Trigger isEjecting() {
        return currentGoal.is(RobotGoal.EJECTING);
    }

    @Override
    public Trigger isClimbing() {
        return currentGoal.is(RobotGoal.CLIMBING);
    }

    // ==================== COMPOSED TRIGGERS ====================
    // These combine current goal + selected level

    @Override
    public Trigger scoringAtL1() {
        return isScoring().and(intent.levelIs(ScoringLevel.L1));
    }

    @Override
    public Trigger scoringAtL2() {
        return isScoring().and(intent.levelIs(ScoringLevel.L2));
    }

    @Override
    public Trigger scoringAtL3() {
        return isScoring().and(intent.levelIs(ScoringLevel.L3));
    }

    @Override
    public Trigger scoringAtL4() {
        return isScoring().and(intent.levelIs(ScoringLevel.L4));
    }

    // ==================== ACCESSORS ====================

    /** Get the current robot goal. */
    public RobotGoal getCurrentGoal() {
        return currentGoal.get();
    }

    /** Get the operator intent for direct access if needed. */
    public OperatorIntent getIntent() {
        return intent;
    }
}
