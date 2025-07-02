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

import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffectorIOSim;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffectorIOTalonFX;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.arm.ArmJointIOSim;
import frc.robot.subsystems.arm.ArmJointIOTalonFX;
import frc.robot.subsystems.arm.constants.ElbowConstants;
import frc.robot.subsystems.arm.constants.ShoulderConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorIOSim;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import static frc.robot.subsystems.vision.VisionConstants.limelightFrontName;
import static frc.robot.subsystems.vision.VisionConstants.limelightLeftName;
import static frc.robot.subsystems.vision.VisionConstants.limelightRightName;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraFront;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraLeft;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraRight;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefPositionsUtil;
import frc.robot.util.ReefPositionsUtil.AutoAlignSide;
import frc.robot.util.ReefPositionsUtil.DeAlgaeLevel;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final double DRIVE_SPEED = 1.0;
  private final double ANGULAR_SPEED = 0.75;
  private final boolean INVERT_ENDGAME = true;
  private final Wrist wrist;

  private final ArmJoint shoulder;
  private final ArmJoint elbow;

  private final Elevator elevator;

  private final CoralEndEffector coralEndEffector;

  private final AlgaeEndEffector algaeEndEffector;

  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController co_controller = new CommandXboxController(1);
  private final CommandXboxController characterizeController = new CommandXboxController(2);
  private final CommandXboxController testcontroller = new CommandXboxController(3);

  private final AprilTagVision vision;

  private RobotState robotState;
  private ReefPositionsUtil reefPositions;

  private boolean m_TeleopInitialized = false;

  final LoggedTunableNumber setElevatorDistance = new LoggedTunableNumber("RobotState/Elevator/setDistance", 58);
  final LoggedTunableNumber setWristAngle = new LoggedTunableNumber("RobotState/Wrist/setAngle", -90);
  final LoggedTunableNumber setToesiesVolts = new LoggedTunableNumber("RobotState/Toesies/setVolts", 2);
  final LoggedTunableNumber setFingeysVolts = new LoggedTunableNumber("RobotState/Fingeys/setVolts", 2);
  final LoggedTunableNumber setIntakeVolts = new LoggedTunableNumber("RobotState/Intake/setVolts", 4);
  final LoggedTunableNumber setShoulderAngle = new LoggedTunableNumber("RobotState/Shoulder/setAngle", 0);
  final LoggedTunableNumber setElbowAngle = new LoggedTunableNumber("RobotState/Elbow/setAngle", 0);
  final LoggedTunableNumber setClimberVolts = new LoggedTunableNumber("RobotState/Climber/setVolts", 4);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(){

    CanDef.Builder canivoreCanBuilder = CanDef.builder().bus(CanBus.CANivore);
    CanDef.Builder rioCanBuilder = CanDef.builder().bus(CanBus.Rio);

    switch (Constants.currentMode) {
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurement,
                drive::addVisionMeasurementAutoAlign,
                new VisionIOPhotonVisionSim(limelightLeftName, robotToCameraLeft, drive::getPose),
                new VisionIOPhotonVisionSim(limelightRightName, robotToCameraRight, drive::getPose),
                new VisionIOPhotonVisionSim(limelightFrontName, robotToCameraFront, drive::getPose));

        wrist = new Wrist(new WristIOSim(3));
        elevator = new Elevator(
          new ElevatorIOSim(
            4,
            new ElevatorSim(
              LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60Foc(2), 
                Pounds.of(45).in(Kilograms),
                Inches.of(Elevator.SPOOL_RADIUS).in(Meters), 
                Elevator.REDUCTION
              ), 
              DCMotor.getKrakenX60Foc(2), 
              Inches.of(0).in(Meters),
              Inches.of(32).in(Meters),
              true, 
              Inches.of(0).in(Meters)
            )
          )
        );

        shoulder = new ArmJoint(new ArmJointIOSim(new ShoulderConstants()),Optional.empty());
        elbow = new ArmJoint(new ArmJointIOSim(new ElbowConstants()),Optional.of(shoulder));
        coralEndEffector = new CoralEndEffector(new CoralEndEffectorIOSim(121));
        algaeEndEffector = new AlgaeEndEffector(new AlgaeEndEffectorIOSim(12));
        climber = new Climber(new ClimberIOSim(19));
        
        SmartDashboard.putData(drive);
      break;
      
      //real is default because it is safer
      default:
      //TODO: add back in dummy drive for replay and add REAL case back
      case REAL:
        drive =
        new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurement,
                drive::addVisionMeasurementAutoAlign,
                new VisionIOLimelight(limelightLeftName, drive::getRotation),
                new VisionIOLimelight(limelightRightName, drive::getRotation),
                new VisionIOLimelight(limelightFrontName, drive::getRotation));

        wrist = new Wrist(new WristIOTalonFX(canivoreCanBuilder.id(11).build(),canivoreCanBuilder.id(15).build()));

        elevator = new Elevator(new ElevatorIOTalonFX(rioCanBuilder.id(13).build(),rioCanBuilder.id(14).build()));

        shoulder = new ArmJoint(new ArmJointIOTalonFX(new ShoulderConstants(), InvertedValue.CounterClockwise_Positive, SensorDirectionValue.Clockwise_Positive, NeutralModeValue.Brake), Optional.empty());
        elbow = new ArmJoint(new ArmJointIOTalonFX(new ElbowConstants(), InvertedValue.CounterClockwise_Positive, SensorDirectionValue.Clockwise_Positive, NeutralModeValue.Brake), Optional.of(shoulder));

        coralEndEffector = new CoralEndEffector(new CoralEndEffectorIOTalonFX(canivoreCanBuilder.id(12).build(), canivoreCanBuilder.id(17).build()));
        // coralEndEffector = new CoralEndEffector(new CoralEndEffectorIONova(rioCanBuilder.id(12).build(), canivoreCanBuilder.id(17).build()));

        algaeEndEffector = new AlgaeEndEffector(new AlgaeEndEffectorIOTalonFX(canivoreCanBuilder.id(15).build(), canivoreCanBuilder.id(24).build()));
        // algaeEndEffector = new AlgaeEndEffector(new AlgaeEndEffectorIONova(rioCanBuilder.id(15).build(), canivoreCanBuilder.id(24).build()));

        climber = new Climber(new ClimberIOTalonFX(rioCanBuilder.id(19).build(), INVERT_ENDGAME));

        // Real robot, instantiate hardware IO implementations
        break;

      // default:
      //   drive =
      //       new Drive(
      //           new GyroIO() {},
      //           new ModuleIO() {},
      //           new ModuleIO() {},
      //           new ModuleIO() {},
      //           new ModuleIO() {});

      //   // Replayed robot, disable IO implementations
      //   // (Use same number of dummy implementations as the real robot)
      //   vision =
      //       new AprilTagVision(
      //           drive::setPose, drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

      //   wrist = null;
      //   elevator = null;
      //   shoulder = null;
      //   elbow = null;
      //   fingeys = null;
      //   intake = null;
      //   algaeEndEffector = null;

      //   throw new Exception("The robot is in neither sim nor real. Something has gone seriously wrong");
    }
    reefPositions = ReefPositionsUtil.getInstance();

    // Configure the button bindings
    configureDriverBindings();
    configureCharacterizationButtonBindings();
  }

  public void configureDriverBindings() {

    //#region controller

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
          drive,
          () -> -controller.getLeftY() * DRIVE_SPEED,
          () -> -controller.getLeftX() * DRIVE_SPEED,
          () -> -controller.getRightX() * ANGULAR_SPEED
        )
    );
  }

    // Coral Station Intake Auto Align Sequence†
    

  public void configureCharacterizationButtonBindings() {
    characterizeController
        .back()
        .and(characterizeController.y())
        .whileTrue(drive.sysIdDynamic(Direction.kForward));
    characterizeController
        .back()
        .and(characterizeController.x())
        .whileTrue(drive.sysIdDynamic(Direction.kReverse));
    characterizeController
        .start()
        .and(characterizeController.y())
        .whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    characterizeController
        .start()
        .and(characterizeController.x())
        .whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

    characterizeController.povUp()
      .whileTrue(DriveCommands.wheelRadiusCharacterization(drive))
      .onFalse(DriveCommands.brakeDrive(drive));

    characterizeController
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  SignalLogger.setPath("/media/sda1/logs");
                  // SignalLogger.enableAutoLogging(true);
                  SignalLogger.start();
                  System.out.println("Started Logger");
                }));
    characterizeController
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  SignalLogger.stop();
                  System.out.println("Stopped Logger");
                }));
    
  }
  

  public void teleopInit() {
    if (!this.m_TeleopInitialized) {
      // Only want to initialize starting position once (if teleop multiple times dont reset pose
      // again)
      vision.updateStartingPosition();
      // Turn on updating odometry based on Apriltags
      vision.enableUpdateOdometryBasedOnApriltags();
      m_TeleopInitialized = true;
      SignalLogger.setPath("/media/sda1/");
      SignalLogger.start();
    }
  }

  public void loggingPeriodic() {
    Logger.recordOutput("ReefPositions/Selected Score Position", reefPositions.getScoreLevel());
    Logger.recordOutput("ReefPositions/Selected Auto Align Side", reefPositions.getAutoAlignSide());
    Logger.recordOutput("ReefPositions/Selected DeAlgae Position", reefPositions.getDeAlgaeLevel());
    Logger.recordOutput("ReefPositions/ScorePos/L1", reefPositions.isSelected(ScoreLevel.L1));
    Logger.recordOutput("ReefPositions/ScorePos/L2", reefPositions.isSelected(ScoreLevel.L2));
    Logger.recordOutput("ReefPositions/ScorePos/L3", reefPositions.isSelected(ScoreLevel.L3));
    Logger.recordOutput("ReefPositions/ScorePos/L4", reefPositions.isSelected(ScoreLevel.L4));
    Logger.recordOutput("ReefPositions/AutoAlignSide/Left", reefPositions.isSelected(AutoAlignSide.Left));
    Logger.recordOutput("ReefPositions/AutoAlignSide/Right", reefPositions.isSelected(AutoAlignSide.Right));
    Logger.recordOutput("ReefPositions/DeAlgaePos/Top", reefPositions.isSelected(DeAlgaeLevel.Top));
    Logger.recordOutput("ReefPositions/DeAlgaePos/Low", reefPositions.isSelected(DeAlgaeLevel.Low));
    Logger.recordOutput("ReefPositions/isAutoAlignEnabled", reefPositions.getIsAutoAligning());
  }

  public void disabledPeriodic() {
    // climber.setServoTarget(Degrees.of(180.0));
  }

public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
}
}
