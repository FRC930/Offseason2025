package frc.robot.subsystems.coralendeffector;

import frc.robot.goals.RobotGoalEvents;
import frc.robot.subsystems.elevator.ElevatorEvents;
import frc.robot.util.SubsystemBehavior;

/**
 * Behavior that drives CoralEndEffector state based on robot goals.
 * Reacts to goal events and sensor feedback to intake, score, or eject coral.
 */
public class CoralEndEffectorBehavior extends SubsystemBehavior {

    private final CoralEndEffector coralEndEffector;

    public CoralEndEffectorBehavior(CoralEndEffector coralEndEffector) {
        super();
        this.coralEndEffector = coralEndEffector;
    }

    @Override
    public void configure(RobotGoalEvents goals, CoralEndEffectorEvents coralEndEffector, ElevatorEvents elevator) {
        // Intake when intaking goal is active and elevator is at stow
        goals.isIntaking().and(elevator.atStow()).whileTrue(this.coralEndEffector.intake());

        // Score when scoring goal is active, elevator at target, and has coral
        goals.isScoring()
                .and(elevator.atTarget())
                .and(coralEndEffector.hasCoral())
                .whileTrue(this.coralEndEffector.score());

        // Eject when ejecting goal is active and elevator is at stow
        goals.isEjecting().and(elevator.atStow()).whileTrue(this.coralEndEffector.eject());
    }
}
