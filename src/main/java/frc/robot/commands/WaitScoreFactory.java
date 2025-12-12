package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;

public class WaitScoreFactory {
    public static Command create(Command scoreCommand, double timeoutElevator, double timeoutEject, CoralEndEffector cee) {
        return scoreCommand.withTimeout(timeoutElevator).andThen(cee.setStateScoring().withTimeout(timeoutEject));
    }
}
