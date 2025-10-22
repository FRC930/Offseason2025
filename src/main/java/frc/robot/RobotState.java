package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.VirtualSubsystem;

import org.littletonrobotics.junction.Logger;

public class RobotState extends VirtualSubsystem {
  private static RobotState instance;

  private MutDistance elevatorHeight = Inches.mutable(0);
  private MutAngle shoulderAngle = Degrees.mutable(0);
  private MutAngle elbowAngle = Degrees.mutable(0);
  private MutAngle wristTwist = Degrees.mutable(0);

  // private final LoggedTunableNumber elevatorHeightTune =
      // new LoggedTunableNumber("robotState/elevatorHeight", 0);
  // private final LoggedTunableNumber shoulderAngleTune =
  //     new LoggedTunableNumber("robotState/elbowAngleZ", 0);
  // private final LoggedTunableNumber elbowAngleTune =
  //     new LoggedTunableNumber("robotState/elbowAngleX", 0);
  // private final LoggedTunableNumber wristTwistTune =
  //     new LoggedTunableNumber("robotState/elbowAngleY", 0);

  private final Mechanism2d primaryMechanism2d;

  private final MechanismRoot2d primaryMechanismRoot;
  private final MechanismLigament2d elevatorLigament2d;

  private final MechanismRoot2d robotBaseRoot;
  private final MechanismLigament2d baseLigament2d = new MechanismLigament2d("RobotBase", 150, 0, 24, new Color8Bit(Color.kBlue));

  private MutAngle testStuff = Degrees.mutable(0);

  private final String key;

  private RobotState(String key) {
    this.key = key;

    primaryMechanism2d = new Mechanism2d(500, 300);
    elevatorLigament2d = new MechanismLigament2d("ElevatorLigament", elevatorHeight.in(Centimeters), 90);
    
    robotBaseRoot = primaryMechanism2d.getRoot("2dBaseRoot", 225, 20);
    robotBaseRoot.append(baseLigament2d);

    primaryMechanismRoot = primaryMechanism2d.getRoot("2dPrimary", 300, 20);
    primaryMechanismRoot.append(elevatorLigament2d); 

    SmartDashboard.putData("Mech2d",primaryMechanism2d);

    // shoulder 18in
    // elbow 15in
  }

  public static RobotState instance() {
    if (instance == null) {
      instance = new RobotState("measured");
    }
    return instance;
  }

  @Override
  public void periodic() {
    visualize();
  }

  public Distance getElevatorHeight() {
    return elevatorHeight;
  }

  public void setElevatorHeight(Distance elevatorHeight) {
    this.elevatorHeight.mut_replace(elevatorHeight);
  }

  public void setElevatorSource(MutDistance elevatorHeight) {
    this.elevatorHeight = elevatorHeight;
  }

  int counter = 0;

  private void visualize() {
    Pose3d elevatorPose =
        new Pose3d(ELEVATOR_ATTACH_OFFSET.getTranslation(), ELEVATOR_ATTACH_OFFSET.getRotation())
            .transformBy(
                new Transform3d(
                    new Translation3d(
                        0, 0, -this.elevatorHeight.in(Meters)),
                    new Rotation3d()));

    elevatorLigament2d.setLength(elevatorHeight.in(Centimeters) + 103.5);
    
    Logger.recordOutput("RobotState/Elevator/" + key, elevatorPose);
  }

  private static final Transform3d ELEVATOR_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(2.125), Inches.of(-11.5), Inches.of(3.5)),
          new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(90)));
}
