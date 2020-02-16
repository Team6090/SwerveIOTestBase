/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveTrain;

import frc.robot.commands.Rotate;
import net.bancino.robotics.swerveio.exception.SwerveException;
import net.bancino.robotics.swerveio.exception.SwerveRuntimeException;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.RunnableCommand;

import net.bancino.robotics.swerveio.gyro.NavXGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistributionPanel;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final XboxController xbox0 = new XboxController(0);

  /* The robot's subsystems and commands are defined here */
  private final DriveTrain drivetrain;

  /* Additional global objects can go here. */
  private final PowerDistributionPanel pdp = new PowerDistributionPanel(Const.CAN.POWER_DISTRIBUTION_PANEL);
  private final NavXGyro gyro = new NavXGyro(SPI.Port.kMXP);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    /* Construct our subsystems here if they throw exceptions. */
    try {
      drivetrain = new DriveTrain(gyro);
    } catch (SwerveException e) {
      throw new SwerveRuntimeException(e);
    }

    // Configure the button bindings
    configureButtonBindings();
    configureCommands();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton xbox0A = new JoystickButton(xbox0, XboxController.Button.kA.value);
    xbox0A.whenPressed(new Rotate(drivetrain));

    JoystickButton xbox0X = new JoystickButton(xbox0, XboxController.Button.kX.value);
    xbox0X.whenPressed(new RunnableCommand(() -> {
      drivetrain.setIdleAngle(0, false);
    }, drivetrain));

    JoystickButton xbox0Y = new JoystickButton(xbox0, XboxController.Button.kY.value);
    xbox0Y.whenPressed(new RunnableCommand(() -> {
      drivetrain.setIdleAngle(135, true);
    }, drivetrain));
  }

  private void configureCommands() {
    /* The drivetrain uses three axes: forward, strafe, and angular velocity, in that order. */
    drivetrain.setDefaultCommand(new DriveWithJoystick(drivetrain, gyro, xbox0, XboxController.Axis.kLeftY, XboxController.Axis.kLeftX, XboxController.Axis.kRightX));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This command will run in autonomous
    return null;
  }
}
