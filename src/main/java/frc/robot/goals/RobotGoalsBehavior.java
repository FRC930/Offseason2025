package frc.robot.goals;

import frc.robot.operator.OperatorIntentEvents;
import frc.robot.util.GoalBehavior;

/**
 * Wires operator button presses to robot goal state.
 *
 * This is the teleop-specific logic. Autonomous bypasses this
 * and calls RobotGoals.setGoal() / setLevel() directly.
 */
public class RobotGoalsBehavior extends GoalBehavior {

    private RobotGoals goals;

    public RobotGoalsBehavior(RobotGoals goals) {
        this.goals = goals;
    }

    @Override
    public void configure(OperatorIntentEvents intent) {
        intent.selectL1().onTrue(goals.setLevel(ScoringLevel.L1));
        intent.selectL2().onTrue(goals.setLevel(ScoringLevel.L2));
        intent.selectL3().onTrue(goals.setLevel(ScoringLevel.L3));
        intent.selectL4().onTrue(goals.setLevel(ScoringLevel.L4));

        // Intent triggers → Goal state
        intent.wantsToScore().onTrue(goals.setGoal(RobotGoal.SCORING)).onFalse(goals.setGoal(RobotGoal.IDLE));

        intent.wantsToIntake().whileTrue(goals.setGoal(RobotGoal.INTAKING)).onFalse(goals.setGoal(RobotGoal.IDLE));

        intent.wantsToEject().whileTrue(goals.setGoal(RobotGoal.EJECTING)).onFalse(goals.setGoal(RobotGoal.IDLE));
    }
}
