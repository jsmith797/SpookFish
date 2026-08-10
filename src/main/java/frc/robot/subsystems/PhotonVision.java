package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.apriltag.*;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class PhotonVision extends SubsystemBase {

 
  private final String MedusaCamera1Name = "Arducam_OV9782_USB_Camera";
  private final PhotonCamera MedusaCamera1;
  
  private double lastAzimuth = 0; 
  private double lastDistance = 2.0; 
  private final Pose2d targetPose = new Pose2d(11.88, 7.42, new Rotation2d()); // Location of target in field

  private Transform3d MedusaCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-14),
              Units.inchesToMeters(11),
              Units.inchesToMeters(0)),
          new Rotation3d(
              0,
              Units.degreesToRadians(0),
              Units.degreesToRadians(0)));

  private Pose2d leftPose2d;
  private String fiducial = "";
  private String leftPosition = "(N/A)";

  private AprilTagFieldLayout fieldLayout;

  // SIMULATION 
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;
  private Supplier<Pose2d> simPoseSupplier;

  // CONSTRUCTOR 
  public PhotonVision() {

    MedusaCamera1 = new PhotonCamera(MedusaCamera1Name);

    fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // SIM CONFIG (Only runs when sim is active)
    if (RobotBase.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(fieldLayout);

      SimCameraProperties cameraProps = new SimCameraProperties();
      cameraProps.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      cameraProps.setFPS(30);
      cameraProps.setAvgLatencyMs(30);
      cameraProps.setLatencyStdDevMs(5);

      cameraSim = new PhotonCameraSim(MedusaCamera1, cameraProps);
      visionSim.addCamera(cameraSim, MedusaCameraTransform);
    }
  }

  // Hook for drivetrain pose
  public void setSimPoseSupplier(Supplier<Pose2d> supplier) {
    this.simPoseSupplier = supplier;
  }

  //PERIODIC
  @Override
  public void periodic() {

    //SIM UPDATE
    if (RobotBase.isSimulation() && simPoseSupplier != null) {
      visionSim.update(simPoseSupplier.get());
    }

    //UPDATE DASHHBOARD
    leftPose2d = updateCamera(MedusaCamera1, MedusaCameraTransform);
    if (leftPose2d != null) {
      leftPosition = getFomattedPose(leftPose2d);
      SmartDashboard.putString("Left:" + fiducial + " ", leftPosition);
    } else {
      SmartDashboard.putString("Left:", "None");
    }
    getDistanceToTarget(); 
    getAzimuthToTarget();
  
  }

  public double getDistanceToTarget() {
    // Returns pythagorean distance from camera center to target. 
    Pose2d cameraPose = this.getRobotPosition(); 

    if (cameraPose != null) {
        lastDistance = cameraPose.getTranslation().getDistance(targetPose.getTranslation());
        SmartDashboard.putBoolean("Vision/Has Target", true);
    } else {
        SmartDashboard.putBoolean("Vision/Has Target", false);
    }
    
    SmartDashboard.putNumber("Vision/Distance", lastDistance);
    return lastDistance;
}

public double getAzimuthToTarget() {
    Pose2d cameraPose = this.getRobotPosition();
    
    if (cameraPose != null && targetPose != null) {
        // Calculate the vector from robot to target
        double deltaX = targetPose.getX() - cameraPose.getX();
        double deltaY = targetPose.getY() - cameraPose.getY();

        // atan2(y, x) gives the absolute angle from the field origin to the target
        Rotation2d angleToTarget = new Rotation2d(deltaX, deltaY);
        
        // To get the RELATIVE azimuth, subtract the robot's current heading
        Rotation2d relativeAzimuth = angleToTarget.minus(cameraPose.getRotation());

        lastAzimuth = relativeAzimuth.getRadians();
        SmartDashboard.putBoolean("Vision/Has Rotation", true);
    } else {
        SmartDashboard.putBoolean("Vision/Has Rotation", false);
    }

    SmartDashboard.putNumber("Vision/Azimuth", lastAzimuth);    
    return lastAzimuth;
  }



  

  //CAMERA PROCESSING
  private Pose2d updateCamera(PhotonCamera camera, Transform3d cameraTransform) {

    Pose2d latest = null;

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    for (var result : results) {

      if (!result.hasTargets()) continue;

      PhotonTrackedTarget tar = result.getBestTarget();
      if (tar == null) continue;

      Transform3d c2t = tar.getBestCameraToTarget();

      Optional<Pose3d> tagPose =
          fieldLayout.getTagPose(tar.getFiducialId());

      fiducial = String.valueOf(tar.getFiducialId());

      if (tagPose.isEmpty() || tar.getPoseAmbiguity() > 0.15) continue;

      Pose2d p =
          PhotonUtils.estimateFieldToRobotAprilTag(
                  c2t,
                  tagPose.get(),
                  cameraTransform.inverse())
              .toPose2d();

      SmartDashboard.putString(
          camera.getName() + "-cam",
          p.getX()
              + " - "
              + p.getY()
              + " - "
              + Units.radiansToDegrees(p.getRotation().getRadians()));

      latest = p;
    }

    return latest;
  }

  // PUBLIC ACCESS 
  public Pose2d getRobotPosition() {
    return leftPose2d;
  }

  //FORMAT
  private String getFomattedPose(Pose2d pose) {

    if (pose == null) return "(N/A)";

    return String.format(
        "(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  
}