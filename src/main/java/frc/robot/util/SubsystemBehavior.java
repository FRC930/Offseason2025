package frc.robot.util;

import frc.robot.goals.RobotGoalEvents;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorEvents;
import frc.robot.subsystems.elevator.ElevatorEvents;
import java.util.ArrayList;
import java.util.List;

/**
 * Base class for hardware subsystem behaviors that react to robot goals.
 * These behaviors translate robot goals into hardware subsystem state changes.
 *
 * <p>Subsystem behaviors listen to the API layer (RobotGoalEvents) and drive
 * the hardware layer (Elevator, CoralEndEffector, etc.).
 *
 * <p>Usage:
 * <pre>{@code
 * public class ElevatorBehavior extends SubsystemBehavior {
 *     private final Elevator elevator;
 *
 *     public ElevatorBehavior(Elevator elevator) {
 *         super();
 *         this.elevator = elevator;
 *     }
 *
 *     @Override
 *     public void configure(RobotGoalEvents goals, ...) {
 *         goals.scoringAtL1().whileTrue(elevator.setState(ElevatorState.L1));
 *     }
 * }
 * }</pre>
 */
public abstract class SubsystemBehavior extends Behavior<SubsystemBehavior> {

    private static final List<SubsystemBehavior> subsystemBehaviors = new ArrayList<>();

    protected SubsystemBehavior() {
        super();
        subsystemBehaviors.add(this);
    }

    /**
     * Configure all registered subsystem behaviors.
     *
     * @param goals Robot goal events to react to
     * @param coralEndEffector Coral end effector events for sensor feedback
     * @param elevator Elevator events for position feedback
     */
    public static void configureAll(
            RobotGoalEvents goals, CoralEndEffectorEvents coralEndEffector, ElevatorEvents elevator) {
        for (SubsystemBehavior behavior : subsystemBehaviors) {
            behavior.configure(goals, coralEndEffector, elevator);
        }
    }

    /**
     * Configure this subsystem behavior's trigger bindings.
     *
     * @param goals Robot goal events to react to
     * @param coralEndEffector Coral end effector events for sensor feedback
     * @param elevator Elevator events for position feedback
     */
    public abstract void configure(
            RobotGoalEvents goals, CoralEndEffectorEvents coralEndEffector, ElevatorEvents elevator);
}
