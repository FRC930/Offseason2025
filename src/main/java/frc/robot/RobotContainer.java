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



import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CEEIntakeSpin;
import frc.robot.commands.CEEScoreSpin;
import frc.robot.commands.CEEStop;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreL1Command;
import frc.robot.commands.ScoreL2Command;
import frc.robot.commands.ScoreL3Command;
import frc.robot.commands.ScoreL4Command;
import frc.robot.generated.TunerConstants;
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
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.LoggedTunableNumber;

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

  private final Elevator elevator;

  private final CoralEndEffector cee;



 

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController characterizeController = new CommandXboxController(2);

  private final AprilTagVision vision;

  


  private boolean m_TeleopInitialized = false;

  private AutoCommandManager m_AutoCommandManager;

  final LoggedTunableNumber setElevatorDistance = new LoggedTunableNumber("RobotState/Elevator/setDistance", 58);
  final LoggedTunableNumber setFingeysVolts = new LoggedTunableNumber("RobotState/CoralEndefector/setVolts", 2);
  final LoggedTunableNumber setIntakeVolts = new LoggedTunableNumber("RobotState/Intake/setVolts", 4);
  
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
                new VisionIOPhotonVisionSim(limelightRightName, robotToCameraRight, drive::getPose));
                

       
        //elevator = new Elevator(
          //new ElevatorIOSim(
          // 4,
          //  new ElevatorSim(
          //    LinearSystemId.createElevatorSystem(
          //      DCMotor.getKrakenX60Foc(2), 
          //      Pounds.of(45).in(Kilograms),
          //      Inches.of(Elevator.SPOOL_RADIUS).in(Meters), 
          //      Elevator.REDUCTION
          //    ), 
          //    DCMotor.getKrakenX60Foc(2), 
          //    Inches.of(0).in(Meters),
          //    Inches.of(32).in(Meters),
          //    true, 
          //   Inches.of(0).in(Meters)
          //  )
         // )
        //);
        
        //TODO: uncomment simulator when 3D sim is implemented
        //cee = new CoralEndEffector(new CoralEndEffectorIOSim(121));
        cee = new CoralEndEffector(new CoralEndEffectorIOTalonFX(rioCanBuilder.id(9).build(), rioCanBuilder.id(21).build()));
        elevator = new Elevator(new ElevatorIOTalonFX(rioCanBuilder.id(13).build(),rioCanBuilder.id(14).build()));

      

        
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
                new VisionIOLimelight(limelightRightName, drive::getRotation));
                

      

        elevator = new Elevator(new ElevatorIOTalonFX(rioCanBuilder.id(13).build(),rioCanBuilder.id(14).build()));


    
        cee = new CoralEndEffector(new CoralEndEffectorIOTalonFX(rioCanBuilder.id(9).build(), rioCanBuilder.id(21).build()));
    

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
    m_AutoCommandManager = new AutoCommandManager(cee);
    
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
    
    controller.povDown().onTrue(new ScoreL1Command(elevator));
    controller.povRight().onTrue(new ScoreL2Command(elevator));
    controller.povLeft().onTrue(new ScoreL3Command(elevator));
    controller.povUp().onTrue(new ScoreL4Command(elevator));
    
    controller.rightTrigger()
      .onTrue(new CEEScoreSpin(cee))
      .onFalse(new CEEStop(cee));
    
    controller.leftTrigger().and(cee.getNewNoCoralSupplier())
      .onTrue(new CEEIntakeSpin(cee))
      .onFalse(new CEEStop(cee));
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

  }

  public void disabledPeriodic() {
    // climber.setServoTarget(Degrees.of(180.0));
  }

public Command getAutonomousCommand() {
    return m_AutoCommandManager.getAutonomousCommand();
  }
}
