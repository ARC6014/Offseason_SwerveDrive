// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and
 * {@link #allianceFlip(Pose2d)}
 * methods to flip these values based on the current alliance color.
 */
public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0); // measure of any tape found on the field
    public static final double aprilTagWidth = Units.inchesToMeters(6.0);

    // Dimensions for community and charging station, including the tape.
    public static final class Community {
        // Region dimensions

    }
}
