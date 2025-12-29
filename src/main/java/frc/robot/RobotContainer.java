// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here. *
 */
package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static frc.robot.subsystems.vision.VisionConstants.limelightLeftName;
import static frc.robot.subsystems.vision.VisionConstants.limelightRightName;
import static frc.robot.subsystems.vision.VisionConstants.questCamName;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraLeft;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraRight;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.goals.RobotGoals;
import frc.robot.goals.RobotGoalsBehavior;
import frc.robot.operator.OperatorIntent;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorBehavior;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorBehavior;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOQuest;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.GoalBehavior;
import frc.robot.util.SubsystemBehavior;

public final class RobotContainer {
    // Drive speed constants
    private static final double DRIVE_SPEED = 0.75;
    private static final double ANGULAR_SPEED = 0.75;

    // Layer 1: UI State
    private final OperatorIntent intent = new OperatorIntent(0);

    // Layer 2: Robot Goals
    private final RobotGoals goals;

    // Layer 3: Hardware Subsystems
    private final Drive drive;
    private final Elevator elevator;
    private final CoralEndEffector cee;

    // Vision and Auto
    private final AprilTagVision vision;
    private final AutoCommandManager autoCommandManager;

    // Characterization controller (separate from intent)
    private final CommandXboxController characterizeController = new CommandXboxController(2);

    private boolean m_TeleopInitialized = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        CanDef.Builder rioCanBuilder = CanDef.builder().bus(CanBus.Rio);

        switch (Constants.currentMode) {
            case SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                vision = new AprilTagVision(
                        drive::setPose,
                        drive::addVisionMeasurement,
                        drive::addVisionMeasurementAutoAlign,
                        new VisionIOPhotonVisionSim(limelightLeftName, robotToCameraLeft, drive::getPose),
                        new VisionIOPhotonVisionSim(limelightRightName, robotToCameraRight, drive::getPose));

                elevator = new Elevator(new ElevatorIOSim(
                        4,
                        new ElevatorSim(
                                LinearSystemId.createElevatorSystem(
                                        DCMotor.getKrakenX60Foc(2),
                                        Pounds.of(45).in(Kilograms),
                                        Inches.of(Elevator.SPOOL_RADIUS).in(Meters),
                                        Elevator.REDUCTION),
                                DCMotor.getKrakenX60Foc(2),
                                Inches.of(0).in(Meters),
                                Inches.of(63).in(Meters),
                                true,
                                Inches.of(0).in(Meters))));

                // TODO: uncomment simulator when 3D sim is implemented
                // cee = new CoralEndEffector(new CoralEndEffectorIOSim(121));
                cee = new CoralEndEffector(new CoralEndEffectorIOTalonFX(
                        rioCanBuilder.id(9).build(), rioCanBuilder.id(21).build()));

                SmartDashboard.putData(drive);
            }

                // real is default because it is safer
            default -> {
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                vision = new AprilTagVision(
                        drive::setPose,
                        drive::addVisionMeasurement,
                        drive::addVisionMeasurementAutoAlign,
                        new VisionIOLimelight(limelightLeftName, drive::getRotation),
                        new VisionIOLimelight(limelightRightName, drive::getRotation),
                        new VisionIOQuest(questCamName));

                elevator = new Elevator(new ElevatorIOTalonFX(
                        rioCanBuilder.id(12).build(), rioCanBuilder.id(11).build()));

                cee = new CoralEndEffector(new CoralEndEffectorIOTalonFX(
                        rioCanBuilder.id(9).build(), rioCanBuilder.id(21).build()));
            }
        }

        // Layer 2: Robot Goals (needs OperatorIntent for composed triggers)
        goals = new RobotGoals();

        // Auto commands
        autoCommandManager = new AutoCommandManager(elevator, cee, goals);

        // Configure drive command
        configureDriveCommand();
        configureCharacterizationButtonBindings();

        // Create behaviors (they auto-register via constructors)
        // Layer 2.5: Goal Behavior (intent -> goals)
        new RobotGoalsBehavior(goals);

        // Layer 4: Subsystem Behaviors (goals -> hardware)
        new ElevatorBehavior(elevator);
        new CoralEndEffectorBehavior(cee);

        // Wire all behaviors
        GoalBehavior.configureAll(intent);
        SubsystemBehavior.configureAll(goals, cee, elevator);
    }

    private void configureDriveCommand() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -intent.getDriver().getLeftY() * DRIVE_SPEED,
                () -> -intent.getDriver().getLeftX() * DRIVE_SPEED,
                () -> -intent.getDriver().getRightX() * ANGULAR_SPEED));
    }

    public void configureCharacterizationButtonBindings() {
        characterizeController.back().and(characterizeController.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));

        characterizeController.back().and(characterizeController.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));

        characterizeController
                .start()
                .and(characterizeController.y())
                .whileTrue(drive.sysIdQuasistatic(Direction.kForward));

        characterizeController
                .start()
                .and(characterizeController.x())
                .whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

        characterizeController
                .povUp()
                .whileTrue(DriveCommands.wheelRadiusCharacterization(drive))
                .onFalse(DriveCommands.brakeDrive(drive));

        characterizeController.a().onTrue(new InstantCommand(() -> {
            SignalLogger.setPath("/media/sda1/logs");
            // SignalLogger.enableAutoLogging(true);
            SignalLogger.start();
            System.out.println("Started Logger");
        }));

        characterizeController.b().onTrue(new InstantCommand(() -> {
            SignalLogger.stop();
            System.out.println("Stopped Logger");
        }));
    }

    public void teleopInit() {
        if (!this.m_TeleopInitialized) {
            vision.updateStartingPosition();
            vision.enableUpdateOdometryBasedOnApriltags();
            m_TeleopInitialized = true;
            SignalLogger.setPath("/media/sda1/");
            SignalLogger.start();
        }
    }

    public void loggingPeriodic() {}

    public void disabledPeriodic() {}

    public Command getAutonomousCommand() {
        return autoCommandManager.getAutonomousCommand();
    }
}
