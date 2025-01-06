package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {

  public record OdometryMeasurement(
      SwerveModulePosition[] wheelPositions, Rotation2d gyro, double timeStamp) {}

  private static final SwerveDriveKinematics kinematics = DriveConstants.kinematics;

  private static SwerveModulePosition[] lastModulePositions;

  private static final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), lastModulePositions, new Pose2d());

  public static void addOdometryMeasurement(OdometryMeasurement measurement) {
    lastModulePositions = measurement.wheelPositions;
    poseEstimator.updateWithTime(
        measurement.timeStamp, measurement.gyro, measurement.wheelPositions);
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "PoseEstimator/Robot")
  public static Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current rotation. */
  public static Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public static SwerveModulePosition[] getLastPositions() {
    return lastModulePositions;
  }

  /** Resets the current odometry pose. */
  public static void setPose(Pose2d pose) {
    poseEstimator
        .resetPosition(
            poseEstimator.getEstimatedPosition().getRotation(),
            lastModulePositions,
            pose);
  }

}
