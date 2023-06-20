// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveByJoystick extends CommandBase {

  private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
  private final DoubleSupplier mX;
  private final DoubleSupplier mY;
  private final DoubleSupplier mRotation;
  private final BooleanSupplier mIsLocked;
  private final BooleanSupplier mRush; // higher speed
  private final BooleanSupplier mSteady; // more controlled/slower

  private final SlewRateLimiter mSlewX = new SlewRateLimiter(DriveConstants.driveSlewRateLimitX);
  private final SlewRateLimiter mSlewY = new SlewRateLimiter(DriveConstants.driveSlewRateLimitY);
  private final SlewRateLimiter mSlewRot = new SlewRateLimiter(DriveConstants.driveSlewRateLimitRot);

  private boolean fieldOriented = DriveConstants.isFieldOriented;
  private double driveScalarValue = DriveConstants.drivePowerScalar;

  /** Creates a new DriveByJoystick. */
  public DriveByJoystick(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, BooleanSupplier isLocked,
      BooleanSupplier rush, BooleanSupplier steady) {
    mX = x;
    mY = y;
    mRotation = rotation;
    mIsLocked = isLocked;
    mRush = rush;
    mSteady = steady;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.lockSwerve(mIsLocked.getAsBoolean());

    double scalar = mRush.getAsBoolean() ? 1 : driveScalarValue;
    if (mSteady.getAsBoolean()) {
      scalar = 0.3;
    } else if (mRush.getAsBoolean()) {
      scalar = 1;
    } else {
      scalar = driveScalarValue;
    }

    double xSpeed = mSlewX.calculate(inputTransform(mX.getAsDouble()) * DriveConstants.maxSpeed) * scalar;
    double ySpeed = mSlewY.calculate(inputTransform(mY.getAsDouble()) * DriveConstants.maxSpeed) * scalar;
    double rotation = mSlewRot
        .calculate(inputTransform(mRotation.getAsDouble()) * DriveConstants.maxAngularSpeedRadPerSec) * scalar;

    if (mIsLocked.getAsBoolean()) {
      mSlewX.reset(0);
      mSlewY.reset(0);
      mSlewRot.reset(0);
    }

    mDrive.swerveDrive(xSpeed, ySpeed, rotation, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double inputTransform(double input) {
    if (input < 0) {
      return -Math.pow(input, 2);
    } else {
      return Math.pow(input, 2);
    }
  }
}
