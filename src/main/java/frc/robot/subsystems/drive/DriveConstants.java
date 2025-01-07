// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class DriveConstants {

  // TODO: get real ID
  public static final int pigeonCANID = 13;

  private static final Distance trackWidthX = Inches.of(21);
  private static final Distance trackWidthY = Inches.of(21);

  public static final Distance wheelRadius = Inches.of(2.1);

  public static final double driveBaseRadius =
      Math.hypot(trackWidthX.in(Meters) / 2.0, trackWidthY.in(Meters) / 2.0);

  // Theoretical free speed (m/s) at 12 V applied output;
  // This needs to be tuned to your individual robot
  public static final double maxLinearSpeed = Units.feetToMeters(19.5);
  public static final double maxAngularVel = maxLinearSpeed / driveBaseRadius;

  public static final Mk4iReductions driveGearRatio = Mk4iReductions.L3_16T;
  public static final Mk4iReductions turnGearRatio = Mk4iReductions.TURN;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  public static final Current slipCurrent = Amps.of(120);

  public static final String CANBusName = "canivore";
  public static final double odometryFrequency = CANBusName.equals("rio") ? 100.0 : 250.0;

  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidthX.times(0.5), trackWidthY.times(0.5)),
        new Translation2d(trackWidthX.times(0.5), trackWidthY.times(-0.5)),
        new Translation2d(trackWidthX.times(-0.5), trackWidthY.times(0.5)),
        new Translation2d(trackWidthX.times(-0.5), trackWidthY.times(-0.5))
      };

  public static final double robotMassKG = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;

  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          robotMassKG,
          robotMOI,
          new ModuleConfig(
              wheelRadius.in(Meters),
              maxLinearSpeed,
              wheelCOF,
              DCMotor.getKrakenX60Foc(1).withReduction(driveGearRatio.reduction),
              driveGearRatio.reduction,
              slipCurrent.in(Amps),
              1),
          moduleTranslations);

  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  public enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L2_16T((50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    L3_16T((50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }

  public static enum ModuleConstants {
    // TODO: get real ID's
    FRONT_LEFT(1, 2, 3, new Rotation2d()),
    FRONT_RIGHT(4, 5, 6, new Rotation2d()),
    BACK_LEFT(7, 8, 9, new Rotation2d()),
    BACK_RIGHT(10, 11, 12, new Rotation2d());

    final int driveCANID;
    final int turnCANID;
    final int encoderCANID;
    final Rotation2d encoderOffset;

    ModuleConstants(int driveCANID, int turnCANID, int encoderCANID, Rotation2d encoderOffset) {
      this.driveCANID = driveCANID;
      this.turnCANID = turnCANID;
      this.encoderCANID = encoderCANID;
      this.encoderOffset = encoderOffset;
    }
  }
}
