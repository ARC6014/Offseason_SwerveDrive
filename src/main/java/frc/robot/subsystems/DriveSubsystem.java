// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.team6014.lib.drivers.SwerveModuleBase;
import frc.team6014.lib.util.LoggedTunableNumber;
import frc.team6014.lib.util.SwerveUtils.SwerveModuleConstants;

public class DriveSubsystem extends SubsystemBase {
  // Swerve numbering:
  // 0 1
  // 2 3

  private static DriveSubsystem mInstance;

  private final Trigger brakeModeTrigger; // ! Learn what these 2 are
  private final StartEndCommand brakeModeCommand;

  public SwerveModuleBase[] mSwerveModules; // collection of modules
  private SwerveModuleState[] states; // collection of modules' states
  private ChassisSpeeds desiredChassisSpeeds; // speeds relative to the robot chassis

  public SwerveDriveOdometry mOdometry;

  private double[] velocityDesired = new double[4];
  private double[] angleDesired = new double[4];

  Pigeon2 mGyro = new Pigeon2(Constants.Pigeon2CanID, Constants.CANIVORE_CANBUS);

  private double snapAngle = 0.0;

  private double timeSinceRot = 0.0;
  private double lastRotTime = 0.0;
  private double timeSinceDrive = 0.0;
  private double lastDriveTime = 0.0;

  private boolean isLocked = false;

  private ProfiledPIDController snapPIDController = new ProfiledPIDController(DriveConstants.snapkP,
      DriveConstants.snapkI, DriveConstants.snapkD, DriveConstants.rotPIDconstraints);

  private final Timer snapTimer = new Timer();

  // TODO: Update periodic for tunable numbers (right now I couldn't do it)
  private static final LoggedTunableNumber front_left_module_kS = new LoggedTunableNumber("FL Module kS",
      Constants.SwerveModuleFrontLeft.modulekS);
  private static final LoggedTunableNumber front_right_module_kS = new LoggedTunableNumber("FR Module kS",
      Constants.SwerveModuleFrontRight.modulekS);
  private static final LoggedTunableNumber rear_left_module_kS = new LoggedTunableNumber("RL Module kS",
      Constants.SwerveModuleRearLeft.modulekS);
  private static final LoggedTunableNumber rear_right_module_kS = new LoggedTunableNumber("RR Module kS",
      Constants.SwerveModuleRearRight.modulekS);

  private static final LoggedTunableNumber front_left_module_kV = new LoggedTunableNumber("FL Module kV",
      Constants.SwerveModuleFrontLeft.modulekV);
  private static final LoggedTunableNumber front_right_module_kV = new LoggedTunableNumber("FR Module kV",
      Constants.SwerveModuleFrontRight.modulekV);
  private static final LoggedTunableNumber rear_left_module_kV = new LoggedTunableNumber("RL Module kV",
      Constants.SwerveModuleRearLeft.modulekV);
  private static final LoggedTunableNumber rear_right_module_kV = new LoggedTunableNumber("RR Module kV",
      Constants.SwerveModuleRearRight.modulekV);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    mSwerveModules = new SwerveModuleBase[] {
        new SwerveModuleBase(0, "FL", SwerveModuleConstants.generateModuleConstants(
            Constants.SwerveModuleFrontLeft.driveMotorID, Constants.SwerveModuleFrontLeft.angleMotorID,
            Constants.SwerveModuleFrontLeft.cancoderID, Constants.SwerveModuleFrontLeft.angleOffset,
            Constants.SwerveModuleFrontLeft.modulekS, Constants.SwerveModuleFrontLeft.modulekV),
            DriveConstants.swerveConstants),

        new SwerveModuleBase(1, "FR", SwerveModuleConstants.generateModuleConstants(
            Constants.SwerveModuleFrontRight.driveMotorID, Constants.SwerveModuleFrontRight.angleMotorID,
            Constants.SwerveModuleFrontRight.cancoderID, Constants.SwerveModuleFrontRight.angleOffset,
            Constants.SwerveModuleFrontRight.modulekS, Constants.SwerveModuleFrontRight.modulekV),
            DriveConstants.swerveConstants),

        new SwerveModuleBase(2, "RL", SwerveModuleConstants.generateModuleConstants(
            Constants.SwerveModuleRearLeft.driveMotorID, Constants.SwerveModuleRearLeft.angleMotorID,
            Constants.SwerveModuleRearLeft.cancoderID, Constants.SwerveModuleRearLeft.angleOffset,
            Constants.SwerveModuleRearLeft.modulekS, Constants.SwerveModuleRearLeft.modulekV),
            DriveConstants.swerveConstants),

        new SwerveModuleBase(3, "RR", SwerveModuleConstants.generateModuleConstants(
            Constants.SwerveModuleRearRight.driveMotorID, Constants.SwerveModuleRearRight.angleMotorID,
            Constants.SwerveModuleRearRight.cancoderID, Constants.SwerveModuleRearRight.angleOffset,
            Constants.SwerveModuleRearRight.modulekS, Constants.SwerveModuleRearRight.modulekV),
            DriveConstants.swerveConstants)
    };

    snapTimer.reset();
    snapTimer.start();

    snapPIDController.enableContinuousInput(-Math.PI, Math.PI); // ensure that the PID controller knows -180 and 180 are
                                                                // connected

    zeroHeading();

    mOdometry = new SwerveDriveOdometry(Constants.kinematics, getRotation2d(), getModulePositions());

    brakeModeTrigger = new Trigger(RobotState::isEnabled);
    brakeModeCommand = new StartEndCommand(() -> {
      for (SwerveModuleBase mod : mSwerveModules) {
        mod.setNeutralMode2Brake(true);
      }
    }, () -> {
      Timer.delay(1.5);
      for (SwerveModuleBase mod : mSwerveModules) {
        mod.setNeutralMode2Brake(false);
      }
    });
  }

  public static DriveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new DriveSubsystem();
    }
    return mInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    brakeModeTrigger.whileTrue(brakeModeCommand);

  }

  /*
   * Manual Swerve Drive Method
   */

  public void swerveDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // if robot is field centric, construct ChassisSpeeds from field relative speeds
    // if not, construct ChassisSpeeds from robot relative speeds
    desiredChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getDriverCentricRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    states = Constants.kinematics.toSwerveModuleStates(desiredChassisSpeeds);

    if (isLocked) {
      states = new SwerveModuleState[] {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed); // normalizes wheel speeds to absolute
                                                                                  // threshold

    /*
     * Sets open loop states
     */
    for (int i = 0; i < 4; i++) {
      mSwerveModules[i].setDesiredState(states[i], true);
      velocityDesired[i] = states[i].speedMetersPerSecond;
      angleDesired[i] = states[i].angle.getDegrees();
    }

  }

  /*
   * Auto Swerve States Method (Closed Loop)
   */

  public synchronized void setClosedLoopStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
    if (isLocked) {
      states = new SwerveModuleState[] {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }
    mSwerveModules[0].setDesiredState(desiredStates[0], false);
    mSwerveModules[1].setDesiredState(desiredStates[1], false);
    mSwerveModules[2].setDesiredState(desiredStates[2], false);
    mSwerveModules[3].setDesiredState(desiredStates[3], false);

  }

  public void setClosedLoopStates(ChassisSpeeds speeds) {
    SwerveModuleState[] desiredStates = Constants.kinematics.toSwerveModuleStates(speeds);
    setClosedLoopStates(desiredStates);
  }

  public void calibrate() {
    for (SwerveModuleBase mod : mSwerveModules) {
      mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
    }
  }

  /*
   * Setters
   */
  public void resetOdometry(Pose2d pose) {
    mGyro.reset();
    mGyro.setYaw(pose.getRotation().times(DriveConstants.invertGyro ? -1 : 1).getDegrees());
    mOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometry(Rotation2d angle) {
    Pose2d pose = new Pose2d(getPoseMeters().getTranslation(), angle);
    mGyro.reset();
    mGyro.setYaw(angle.getDegrees());
    mOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), pose);
  }

  public void resetSnapPID() {
    snapPIDController.reset(getRotation2d().getRadians());
  }

  public void zeroHeading() {
    mGyro.reset();
  }

  public void stop() {
    for (SwerveModuleBase module : mSwerveModules) {
      module.stop();
    }
  }

  public void resetToAbsolute() {
    for (SwerveModuleBase module : mSwerveModules) {
      module.stop();
      module.resetToAbsolute();
    }
  }

  public void lockSwerve(boolean should) {
    isLocked = should;
  }

  public void updateOdometry() {
    mOdometry.update(
        getRotation2d(),
        getModulePositions());
  }

  public void setPose(Pose2d pose) {
    mOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), pose);
  }

  /*
   * Getters
   */

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(mGyro.getAngle(), 360.0))
        .times(DriveConstants.invertGyro ? -1 : 1);
  }

  // Returns gyro angle relative to alliance station
  public Rotation2d getDriverCentricRotation2d() {
    return DriverStation.getAlliance() == Alliance.Red
        ? Rotation2d.fromDegrees(Math.IEEEremainder(mGyro.getAngle() + 180, 360.0))
            .times(DriveConstants.invertGyro ? -1 : 1)
        : Rotation2d.fromDegrees(Math.IEEEremainder(mGyro.getAngle(), 360.0))
            .times(DriveConstants.invertGyro ? -1 : 1);
  }

  public Pose2d getPoseMeters() {
    return mOdometry.getPoseMeters();
  }

  SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        mSwerveModules[0].getPosition(),
        mSwerveModules[1].getPosition(),
        mSwerveModules[2].getPosition(),
        mSwerveModules[3].getPosition(),
    };
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModuleBase mod : mSwerveModules) {
      states[mod.getModuleNumber()] = mod.getState();
    }
    return states;
  }

  public ChassisSpeeds getChassisSpeed() {
    return Constants.kinematics.toChassisSpeeds(mSwerveModules[0].getState(), mSwerveModules[1].getState(),
        mSwerveModules[2].getState(), mSwerveModules[3].getState());
  }
  /*
   * private double calculateSnapValue(double xSpeed, double ySpeed, double rot) {
   * 
   * double output = rot;
   * 
   * if (Math.abs(rot) >= 0.05) {
   * lastRotTime = snapTimer.get();
   * }
   * 
   * if (Math.abs(xSpeed) >= 0.05 || Math.abs(ySpeed) >= 0.05) {
   * lastDriveTime = snapTimer.get();
   * }
   * 
   * timeSinceRot = snapTimer.get() - lastRotTime;
   * timeSinceDrive = snapTimer.get() - lastDriveTime;
   * 
   * if (timeSinceRot < 0.5) {
   * snapAngle = getRotation2d().getRadians();
   * } else if (Math.abs(rot) < 0.05 && timeSinceDrive < 0.2) {
   * output = snapPIDController.calculate(getRotation2d().getRadians(),
   * snapAngle);
   * }
   * return output;
   * }
   */

}
