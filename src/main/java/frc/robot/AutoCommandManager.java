package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.goals.RobotGoal;
import frc.robot.goals.RobotGoals;
import frc.robot.goals.ScoringLevel;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoCommandManager {

    private final LoggedDashboardChooser<Command> autoChooser;

    public AutoCommandManager(Elevator elevator, CoralEndEffector cee, RobotGoals goals) {
        // Note: no OperatorIntent needed!
        configureNamedCommands(elevator, cee, goals);
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    }

    private void configureNamedCommands(Elevator elevator, CoralEndEffector cee, RobotGoals goals) {
        NamedCommands.registerCommand("Stow", goals.setGoal(RobotGoal.IDLE));
        NamedCommands.registerCommand("Intake", goals.setGoal(RobotGoal.INTAKING));

        // Clean! One call sets both level and goal
        NamedCommands.registerCommand("ScoreL1", goals.scoreAt(ScoringLevel.L1));
        NamedCommands.registerCommand("ScoreL2", goals.scoreAt(ScoringLevel.L2));
        NamedCommands.registerCommand("ScoreL3", goals.scoreAt(ScoringLevel.L3));
        NamedCommands.registerCommand("ScoreL4", goals.scoreAt(ScoringLevel.L4));

        NamedCommands.registerCommand(
                "ConfirmScore", cee.score().until(cee.hasCoral().negate()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
