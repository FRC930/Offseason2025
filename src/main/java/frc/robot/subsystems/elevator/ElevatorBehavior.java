package frc.robot.subsystems.elevator;

import frc.robot.goals.RobotGoalEvents;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorEvents;
import frc.robot.util.SubsystemBehavior;

/**
 * Behavior that drives Elevator state based on robot goals.
 * Reacts to goal events to move the elevator to appropriate positions.
 */
public class ElevatorBehavior extends SubsystemBehavior {

    private final Elevator elevator;

    public ElevatorBehavior(Elevator elevator) {
        super();
        this.elevator = elevator;
    }

    @Override
    public void configure(RobotGoalEvents goals, CoralEndEffectorEvents coralEndEffector, ElevatorEvents elevator) {
        // Move to level when scoring at that level
        goals.scoringAtL1().whileTrue(this.elevator.setState(ElevatorState.L1));
        goals.scoringAtL2().whileTrue(this.elevator.setState(ElevatorState.L2));
        goals.scoringAtL3().whileTrue(this.elevator.setState(ElevatorState.L3));
        goals.scoringAtL4().whileTrue(this.elevator.setState(ElevatorState.L4));

        // Stow when intaking, idle, ejecting, or climbing
        goals.isIntaking()
                .or(goals.isIdle())
                .or(goals.isEjecting())
                .or(goals.isClimbing())
                .whileTrue(this.elevator.setState(ElevatorState.STOW));
    }
}
