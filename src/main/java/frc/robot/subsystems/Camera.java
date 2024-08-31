// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera extends SubsystemBase {
  PhotonCamera camera;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  PhotonPoseEstimator photonPoseEstimator;
  // meters
  private final double cameraOffsetX = -0.33655;
  private final double cameraOffsetY = -0.01016;
  private final double cameraOffsetZ = 0.0889;
  // radians
  private final double cameraRoll = 0;
  private final double cameraPitch = 0;
  private final double cameraYaw = 0;
  Transform3d cameraToRobot;
  private PhotonPipelineResult result = new PhotonPipelineResult();
  Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

  public Camera(String name) {
    this.camera = new PhotonCamera(name);
    this.cameraToRobot =
        new Transform3d(
            new Translation3d(cameraOffsetX, cameraOffsetY, cameraOffsetZ),
            new Rotation3d(cameraRoll, cameraPitch, cameraYaw));
    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraToRobot);
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    Optional<EstimatedRobotPose> newPose = photonPoseEstimator.update(result);
    if (newPose.isPresent()) {
      lastEstimatedPose = newPose;
    }
  }

  /**
   * Returns the global position of the robot.
   *
   * @return
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return lastEstimatedPose;
  }

  // This method should ONLY be called by the pose fusion portion of the drive
  // system. Any other calls will invalidate it.
  public Optional<Double> getPoseTimeStamp() {
    if (result.getTimestampSeconds() == -1) {
      return Optional.empty();
    } else {
      return Optional.of(result.getTimestampSeconds());
    }
  }
}
