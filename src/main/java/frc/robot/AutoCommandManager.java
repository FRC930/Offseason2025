package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.goals.RobotGoal;
import frc.robot.goals.RobotGoals;
import frc.robot.operator.OperatorIntent;
import frc.robot.operator.ScoringLevel;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommandManager {

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    public AutoCommandManager(Elevator elevator, CoralEndEffector cee, RobotGoals goals, OperatorIntent intent) {
        configureNamedCommands(elevator, cee, goals, intent);
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureNamedCommands(
            Elevator elevator, CoralEndEffector cee, RobotGoals goals, OperatorIntent intent) {
        NamedCommands.registerCommand("Stow", goals.setGoal(RobotGoal.IDLE));
        NamedCommands.registerCommand(
                "ConfirmScore", cee.score().until(cee.hasCoral().negate()));
        NamedCommands.registerCommand(
                "ScoreL1", Commands.sequence(intent.setLevel(ScoringLevel.L1), goals.setGoal(RobotGoal.SCORING)));
        NamedCommands.registerCommand(
                "ScoreL2", Commands.sequence(intent.setLevel(ScoringLevel.L2), goals.setGoal(RobotGoal.SCORING)));
        NamedCommands.registerCommand(
                "ScoreL3", Commands.sequence(intent.setLevel(ScoringLevel.L3), goals.setGoal(RobotGoal.SCORING)));
        NamedCommands.registerCommand(
                "ScoreL4", Commands.sequence(intent.setLevel(ScoringLevel.L4), goals.setGoal(RobotGoal.SCORING)));
        NamedCommands.registerCommand("Intake", goals.setGoal(RobotGoal.INTAKING));
    }
}
