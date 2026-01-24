package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Swerve Module Constants to be used when creating swerve modules.
 *
 * @param driveMotorID
 * @param angleMotorID
 * @param canCoderID
 * @param angleOffset
 */
public record SwerveModuleConstants(
    int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset) {}
