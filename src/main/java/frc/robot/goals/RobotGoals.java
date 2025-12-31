package frc.robot.goals;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EnumState;
import frc.robot.util.VirtualSubsystem;

/**
 * Central robot state: what we're doing (goal) and how (selections).
 *
 * Can be set by:
 * - Teleop via RobotGoalsBehavior reacting to OperatorIntent
 * - Autonomous via direct command calls
 * - Automatic triggers (e.g., "when piece acquired, switch to HOLDING")
 */
public class RobotGoals extends VirtualSubsystem implements RobotGoalEvents {

    // What is the robot doing?
    private final EnumState<RobotGoal> currentGoal = new EnumState<>("RobotGoals/Goal", RobotGoal.IDLE);

    // How should it do it? (selections/parameters)
    private final EnumState<ScoringLevel> selectedLevel = new EnumState<>("RobotGoals/Level", ScoringLevel.L1);
    private final EnumState<ScoringSide> selectedSide = new EnumState<>("RobotGoals/Side", ScoringSide.LEFT);

    public RobotGoals() {}

    // ==================== GOAL MUTATION ====================

    public Command setGoal(RobotGoal goal) {
        return Commands.runOnce(() -> currentGoal.set(goal));
    }

    public Command setLevel(ScoringLevel level) {
        return Commands.runOnce(() -> selectedLevel.set(level));
    }

    public Command setSide(ScoringSide side) {
        return Commands.runOnce(() -> selectedSide.set(side));
    }

    /** Convenience: set level and goal in one command */
    public Command scoreAt(ScoringLevel level) {
        return Commands.sequence(setLevel(level), setGoal(RobotGoal.SCORING));
    }

    // ==================== GOAL TRIGGERS ====================

    @Override
    public Trigger isIdle() {
        return currentGoal.is(RobotGoal.IDLE);
    }

    @Override
    public Trigger isScoring() {
        return currentGoal.is(RobotGoal.SCORING);
    }

    @Override
    public Trigger isIntaking() {
        return currentGoal.is(RobotGoal.INTAKING);
    }

    @Override
    public Trigger isEjecting() {
        return currentGoal.is(RobotGoal.EJECTING);
    }

    @Override
    public Trigger isClimbing() {
        return currentGoal.is(RobotGoal.CLIMBING);
    }

    // ==================== SELECTION TRIGGERS ====================

    @Override
    public Trigger levelIs(ScoringLevel level) {
        return selectedLevel.is(level);
    }

    @Override
    public Trigger sideIs(ScoringSide side) {
        return selectedSide.is(side);
    }

    // ==================== COMPOSED TRIGGERS ====================

    @Override
    public Trigger scoringAtL1() {
        return isScoring().and(levelIs(ScoringLevel.L1));
    }

    @Override
    public Trigger scoringAtL2() {
        return isScoring().and(levelIs(ScoringLevel.L2));
    }

    @Override
    public Trigger scoringAtL3() {
        return isScoring().and(levelIs(ScoringLevel.L3));
    }

    @Override
    public Trigger scoringAtL4() {
        return isScoring().and(levelIs(ScoringLevel.L4));
    }

    // ==================== ACCESSORS ====================

    public RobotGoal getCurrentGoal() {
        return currentGoal.get();
    }

    public ScoringLevel getSelectedLevel() {
        return selectedLevel.get();
    }

    @Override
    public void periodic() {}
}
