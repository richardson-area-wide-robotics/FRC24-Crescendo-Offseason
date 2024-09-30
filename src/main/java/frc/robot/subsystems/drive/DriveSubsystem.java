// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;

public class DriveSubsystem extends Swerve {

    // Swerve module instances initialized with their respective constants
    private static final MAXSwerveModule frontLeft = new MAXSwerveModule(
            Constants.SwerveDriveConstants.FrontLeftModule.S_MODULE_CONSTANTS);
    private static final MAXSwerveModule frontRight = new MAXSwerveModule(
            Constants.SwerveDriveConstants.FrontRightModule.S_MODULE_CONSTANTS);
    private static final MAXSwerveModule backLeft = new MAXSwerveModule(
            Constants.SwerveDriveConstants.BackLeftModule.S_MODULE_CONSTANTS);
    private static final MAXSwerveModule backRight = new MAXSwerveModule(
            Constants.SwerveDriveConstants.BackRightModule.S_MODULE_CONSTANTS);

    private final Camera m_camera;

    public DriveSubsystem(AHRS m_gyro, Camera camera) {
        super(
            frontLeft,
            frontRight,
            backLeft,
            backRight,
            Constants.SwerveDriveConstants.kDriveKinematics,
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            m_gyro,
            Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond);

        configureAutoBuilder();
        this.m_camera = camera;
    }

    private void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry
            this::getChassisSpeeds, // ChassisSpeeds supplier (ROBOT RELATIVE)
            this::driveRobotRelative, // Drives the robot with ROBOT RELATIVE ChassisSpeeds
            createPathFollowerConfig(), // Configuration for path following
            this::isRedAlliance, // Check for red alliance
            this // Reference to this subsystem
        );
    }

    private HolonomicPathFollowerConfig createPathFollowerConfig() {
        return new HolonomicPathFollowerConfig(
            new PIDConstants(5.28, 0.0, 5), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters
            new ReplanningConfig() // Default path replanning config
        );
    }

  /**
  * Checks if the robot is part of the red alliance.
  *
  * @return true if the robot is on the red alliance, false otherwise.
  */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        addChild("X Controller", Constants.AutoConstants.kPXController);
        addChild("Y Controller", Constants.AutoConstants.kPYController);
        addChild("Theta Controller", Constants.AutoConstants.kPThetaController);
    }

    @Override
    public void periodic() {
        updateSmartDashboard();
        super.periodic();
    }

    private void updateSmartDashboard() {
        SmartDashboard.putString("swervepose", this.getPose().toString());

        // Uncomment to include camera pose estimation
        // Optional<EstimatedRobotPose> pose = m_camera.getEstimatedGlobalPose();
        // Optional<Double> time = m_camera.getPoseTimeStamp();
        // SmartDashboard.putBoolean("pose.isPresent()", pose.isPresent());
        // SmartDashboard.putBoolean("time.ispresent", time.isPresent());
        // if (pose.isPresent() && time.isPresent()) {
        //     this.addPoseEstimate(pose.get().estimatedPose.toPose2d(), time.get());
        // }
    }
}
