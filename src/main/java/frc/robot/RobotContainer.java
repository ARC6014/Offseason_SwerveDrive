// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveByJoystick;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
  // private final Joystick mDriver = new Joystick(0);
  // private final Joystick mOperator = new Joystick(1);

  private final CommandXboxController mDriver = new CommandXboxController(0);
  private final CommandJoystick mOperator = new CommandJoystick(1);

  
  //private final DriveByJoystick driveByJoystick = new DriveByJoystick(() -> mDriver.getRawAxis(1) * -1, () -> mDriver.getRawAxis(0) * -1, () -> mDriver.getRawAxis(2) * -1, () -> mDriver.getRawButton(7), () -> mDriver.getRawButton(8), () -> mOperator.getRawButton(9));

  // Alternative if using CommandXboxController
  // TODO: Check lock swerve button functionality
  private final DriveByJoystick driveByJoystick = new DriveByJoystick(mDriver::getLeftY, mDriver::getLeftX,
      mDriver::getRightX, () -> mDriver.leftTrigger().getAsBoolean(), () -> mDriver.rightBumper().getAsBoolean(),
      () -> mDriver.leftBumper().getAsBoolean());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    mDrive.setDefaultCommand(driveByJoystick);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
