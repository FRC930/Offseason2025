package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CoralEndEffectorSpin;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;


public class AutoCommandManager {

  // Dashboard inputs

  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoCommandManager(CoralEndEffector cee) {
    configureNamedCommands(cee);
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

  private void configureNamedCommands(CoralEndEffector cee) {
    // TODO: Implement Stow Command instead of spinning cee.
    NamedCommands.registerCommand("Stow", new CoralEndEffectorSpin(cee));
  };
}
