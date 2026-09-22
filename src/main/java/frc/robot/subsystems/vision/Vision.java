package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// mostly copied from Rebuilt-Java-Rewrite, should work fine
public class Vision extends SubsystemBase {
  private final Drive drive;
  private final String cameraName = "limelight";

  // map groups a string with a value, easier finding
  private final Map<String, Pose2d> mt1Poses = new HashMap<>();
  private final Map<String, List<PoseEstimate>> mt1MeasurementTracker = new HashMap<>();

  private final Field2d visionField = new Field2d();
  private final Map<String, FieldObject2d> cameraFieldObjects = new HashMap<>();

  private boolean sendYawRate = true;
  private double timeDelay = 0.5;
  private int maxFrameCount = 60;

  public Vision(Drive drive, String cameraName) {
    this.drive = drive;

    mt1MeasurementTracker.put(cameraName, new ArrayList<>());
    mt1Poses.put(cameraName, new Pose2d());
    cameraFieldObjects.put(cameraName, visionField.getObject(cameraName));

    visionField.setRobotPose(new Pose2d());
    SmartDashboard.putData(visionField);
  }

  public void setup() {
    double yawDegrees = drive.getRotation().getDegrees();
    double yawDegreesPerSec =
        sendYawRate ? Units.radiansToDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond) : 0.0;
    LimelightHelpers.SetRobotOrientation(cameraName, yawDegrees, yawDegreesPerSec, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(cameraName, 0);
  }

  public void setSendYawRate(boolean sendYawRate) {
    this.sendYawRate = sendYawRate;
  }

  public void periodic() {
    double yawDegrees = drive.getRotation().getDegrees();
    double yawDegreesPerSec =
        sendYawRate ? Units.radiansToDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond) : 0.0;

    boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    List<Pose2d> validPosesThisCycle = new ArrayList<>();

    LimelightHelpers.SetRobotOrientation(cameraName, yawDegrees, yawDegreesPerSec, 0, 0, 0, 0);

    PoseEstimate mt1Estimate =
        isRedAlliance
            ? LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (LimelightHelpers.validPoseEstimate(mt1Estimate)) {
      Matrix<N3, N1> stdDevs = calculateStdDevs(mt1Estimate);

      drive.addVisionMeasurment(mt1Estimate.pose, yawDegreesPerSec, stdDevs);

      mt1Poses.put(cameraName, mt1Estimate.pose);
      mt1MeasurementTracker.get(cameraName).add(mt1Estimate);
      validPosesThisCycle.add(mt1Estimate.pose);

      FieldObject2d cameraObj = cameraFieldObjects.get(cameraName);
      if (cameraObj != null) {
        cameraObj.setPose(mt1Estimate.pose);
      }
    }

    cleanTracker(mt1MeasurementTracker.get(cameraName));
  }

  private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate) {
    if (estimate.tagCount == 0) {
      return VecBuilder.fill(
          Units.inchesToMeters(36), Units.inchesToMeters(36), Units.degreesToRadians(30));
    }

    if (estimate.tagCount >= 2) {
      return VecBuilder.fill(0.3, 0.3, 0.3);
    } else {
      double distance = estimate.avgTagDist;
      double xyStdDev = 0.5 * Math.pow(distance, 2) / 2.0;
      double rotStdDev = 1.0 * Math.pow(distance, 2) / 2.0;
      return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
    }
  }

  private void cleanTracker(List<PoseEstimate> tracker) {
    double now = Timer.getFPGATimestamp();
    tracker.removeIf(m -> (now - m.timestampSeconds) > timeDelay);

    while (maxFrameCount > 0 && tracker.size() > maxFrameCount) {
      tracker.remove(0);
    }
  }
}
