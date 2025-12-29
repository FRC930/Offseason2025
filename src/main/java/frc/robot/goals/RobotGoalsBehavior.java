package frc.robot.goals;

import frc.robot.operator.OperatorIntent;
import frc.robot.util.GoalBehavior;

/**
 * Behavior that wires operator intent to robot goal state changes.
 * This follows the same pattern as subsystem behaviors but operates at the goal layer.
 *
 * <p>Architecture:
 * <pre>
 * OperatorIntent (UI Layer)
 *        │
 *        ▼ (RobotGoalsBehavior listens)
 * RobotGoals (API Layer)
 *        │
 *        ▼ (SubsystemBehaviors listen)
 * Hardware Subsystems
 * </pre>
 */
public class RobotGoalsBehavior extends GoalBehavior {

    private final RobotGoals goals;

    public RobotGoalsBehavior(RobotGoals goals) {
        super();
        this.goals = goals;
    }

    @Override
    public void configure(OperatorIntent intent, RobotGoalEvents goalEvents) {
        // Score: active while button held, returns to IDLE on release
        intent.wantsToScore().onTrue(goals.setGoal(RobotGoal.SCORING)).onFalse(goals.setGoal(RobotGoal.IDLE));

        // Intake: active while button held
        intent.wantsToIntake().onTrue(goals.setGoal(RobotGoal.INTAKING)).onFalse(goals.setGoal(RobotGoal.IDLE));

        // Eject: active while button held
        intent.wantsToEject().onTrue(goals.setGoal(RobotGoal.EJECTING)).onFalse(goals.setGoal(RobotGoal.IDLE));

        // Climb: active while in climb mode
        intent.wantsToClimb().onTrue(goals.setGoal(RobotGoal.CLIMBING)).onFalse(goals.setGoal(RobotGoal.IDLE));
    }
}
