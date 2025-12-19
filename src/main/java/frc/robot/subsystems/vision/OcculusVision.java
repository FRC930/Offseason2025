package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class OcculusVision extends SubsystemBase {

  // Allow robot to turn off updating odometry
  private boolean m_updateOdometryBaseOnApriltags = true;
  private Consumer<Pose2d> m_ResetPose;
  private final VisionConsumer consumer;
  private QuestNav questNav = new QuestNav();

  private boolean m_HasRunAutonomous = false;
  private boolean m_HasSeenTags = false;
    private Pose2d initialPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90));
    private Pose3d initial3DPose = new Pose3d(initialPose);
    questNav.setPose(initial3DPose);
    

  private Trigger m_checkIfAllianceChangedTrigger = null;

  private boolean m_isFirstTime = true;
// resets the aprilTag vision to nothing
  public OcculusVision(VisionConsumer addVision) {
    this.consumer = addVision;
    

    // try {
    //   aprilTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/output.json");
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    //   throw new RuntimeException(e);
    // }
  }


  @Override
  public void periodic() {
    updateVisionPose();
    super.periodic();
  }
  
  private Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
          0.035 // Trust down to 2 degrees rotational
          );

  private final Transform2d ROBOT_TO_QUEST =
      new Transform2d(
          Units.inchesToMeters(0.5), Units.inchesToMeters(9.207), Rotation2d.fromDegrees(90));

  private int questDebug = 0;

  public void updateVisionPose() {

    questNav.commandPeriodic(); // Process command responses

    Logger.recordOutput("QuestNav930/QuestIsConnected", questNav.isConnected());
    Logger.recordOutput("QuestNav930/QuestIsTracking", questNav.isTracking());
    Logger.recordOutput("QuestNav930/QuestDebug", questDebug);

    if (questDebug++ > 50) {
      questDebug = 0;
    }

    // Monitor connection and device status
    if (questNav.isConnected() && questNav.isTracking()) {

      // Get latest pose data
      PoseFrame[] newFrames = questNav.getAllUnreadPoseFrames();
      for (PoseFrame questFrame : newFrames) {
        // Use frame.questPose() and frame.dataTimestamp() with pose estimator

        // Get the pose of the Quest
        // Pose3d questPose = questFrame.questPose();
        Pose3d questPose = questFrame.questPose3d();
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        Logger.recordOutput("QuestNav930/QuestPose", questPose);
        Logger.recordOutput("QuestNav930/Timestamp", timestamp);

        // Transform quest to robot pose
        

        Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
        

        drive.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
      }

      // Quest is connected and tracking - safe to use pose data
    }
}
}
