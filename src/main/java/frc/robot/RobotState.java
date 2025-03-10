// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {

  public record OdometryMeasurement(
      SwerveModulePosition[] wheelPositions, Rotation2d gyro, double timeStamp) {}

  public record VisionMeasurement(Pose2d pose, double timeStamp, Matrix<N3, N1> stdDevs) {}

  private final SwerveDriveKinematics kinematics = DriveConstants.kinematics;

  private final SwerveModulePosition[] startingPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveDrivePoseEstimator poseEstimator;

  private final SwerveDriveOdometry odometry;

  private static RobotState instance;

  private RobotState() {
    poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), startingPositions, new Pose2d());
    odometry =
        new SwerveDriveOdometry(kinematics, new Rotation2d(), startingPositions, new Pose2d());
  }

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  /**
   * Adds the data from an odometry measurement to the pose estimator
   *
   * @param measurement The measurement to be added
   */
  public void addOdometryMeasurement(OdometryMeasurement measurement) {
    odometry.update(measurement.gyro, measurement.wheelPositions);
    poseEstimator.updateWithTime(
        measurement.timeStamp, measurement.gyro, measurement.wheelPositions);
  }

  /**
   * Adds the data from a vision measurement to the pose estimator
   *
   * @param measurement The measurement to be added
   */
  public void addVisionMeasurement(VisionMeasurement measurement) {
    poseEstimator.addVisionMeasurement(
        measurement.pose, measurement.timeStamp, measurement.stdDevs);
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose, SwerveModulePosition[] modulePositions) {
    poseEstimator.resetPosition(pose.getRotation(), modulePositions, pose);
  }
}
