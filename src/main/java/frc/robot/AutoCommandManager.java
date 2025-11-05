package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CEEScoreSpin;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreL1Command;
import frc.robot.commands.ScoreL2Command;
import frc.robot.commands.ScoreL3Command;
import frc.robot.commands.ScoreL4Command;
import frc.robot.commands.StowCommand;
import frc.robot.commands.WaitScoreFactory;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;


public class AutoCommandManager {

  // Dashboard inputs

  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoCommandManager(Elevator elevator, CoralEndEffector cee) {
    configureNamedCommands(elevator, cee);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  
  };

  public Command getAutonomousCommand() {
      return autoChooser.get();
  }

  private void configureNamedCommands(Elevator elevator, CoralEndEffector cee) {
    NamedCommands.registerCommand("Stow", new StowCommand(elevator, cee));
    NamedCommands.registerCommand("ConfirmScore", new CEEScoreSpin(cee));
    NamedCommands.registerCommand("ScoreL1", WaitScoreFactory.create(new ScoreL1Command(elevator), 1.0, 0.5, cee));
    NamedCommands.registerCommand("ScoreL2", WaitScoreFactory.create(new ScoreL2Command(elevator), 1.0, 0.5, cee));
    NamedCommands.registerCommand("ScoreL3", WaitScoreFactory.create(new ScoreL3Command(elevator), 1.0, 0.5, cee));
    NamedCommands.registerCommand("ScoreL4", WaitScoreFactory.create(new ScoreL4Command(elevator), 1.0, 0.5, cee));
    NamedCommands.registerCommand("Intake", new IntakeCommand(cee).withTimeout(Robot.isReal() ? 5.0 : 100.0));
  };
}
