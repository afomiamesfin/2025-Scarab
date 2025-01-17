// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.util.Dashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final Joystick driveJoystick;
  public final Joystick turnJoystick;
  public final Joystick controlPanel;

  private final DrivetrainSubsystem drivetrain;
  // private final VisionSubsystem vision;
  // private final AdvantageScopeSubsystem advantageScope;

  private RobotState robotState = RobotState.getInstance();

  public RobotContainer() {
    driveJoystick = new Joystick(0);
    turnJoystick = new Joystick(1);
    controlPanel = new Joystick(2);

    // not running drive in periodic, arguments don't matter
    drivetrain = new DrivetrainSubsystem(
        driveJoystick::getY,
        // Sideways velocity supplier.
        driveJoystick::getX,
        // Rotation velocity supplier.
        turnJoystick::getX,
        () -> true);// Dashboard.getInstance()::isFieldCentric);

    drivetrain.setDefaultCommand(
      new DriveCommand(
        // Forward velocity supplier.
        driveJoystick::getY,
        // Sideways velocity supplier.
        driveJoystick::getX,
        // Rotation velocity supplier.
        turnJoystick::getX,
        () -> true, // Dashboard.getInstance()::isFieldCentric,
        //Dashboard.getInstance()::isFieldCentric,
        drivetrain
      )
    );

    // vision = new VisionSubsystem();
    // advantageScope = new AdvantageScopeSubsystem(drivetrain);

    configureBindings();
  }

  private void configureBindings() {
    /*
     * Drive Button Bindings
     */

    JoystickButton zeroGyroButton = new JoystickButton(driveJoystick, 8);
    zeroGyroButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading())); // won't work since we can't reset
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new WaitCommand(0);
  }
}
