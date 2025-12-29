package frc.robot.goals;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interface exposing robot goal triggers for behaviors to react to.
 * Behaviors should only depend on this interface, not on RobotGoals directly.
 */
public interface RobotGoalEvents {
    Trigger isIdle();

    Trigger isIntaking();

    Trigger isScoring();

    Trigger isEjecting();

    Trigger isClimbing();

    Trigger levelIs(ScoringLevel level);

    Trigger sideIs(ScoringSide side);

    Trigger scoringAtL1();

    Trigger scoringAtL2();

    Trigger scoringAtL3();

    Trigger scoringAtL4();
}
