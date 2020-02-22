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
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.HallAuton;
import frc.robot.commands.Rotate;
import frc.robot.subsystems.DriveTrain;

import net.bancino.robotics.swerveio.exception.SwerveException;
import net.bancino.robotics.swerveio.exception.SwerveRuntimeException;
import net.bancino.robotics.swerveio.command.SwerveDriveTeleop;
import net.bancino.robotics.swerveio.command.RunnableCommand;

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
  @SuppressWarnings("unused")
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
    JoystickButton leftBumper = new JoystickButton(xbox0, XboxController.Button.kBumperLeft.value);
    leftBumper.whenPressed(new RunnableCommand(() -> {
      drivetrain.getGyro().zero();
    }, drivetrain));

    JoystickButton xbox0X = new JoystickButton(xbox0, XboxController.Button.kX.value);
    xbox0X.whenPressed(new Rotate(drivetrain, gyro, 90, 1000));
  }

  private void configureCommands() {
    /* The drivetrain uses three axes: forward, strafe, and angular velocity, in that order. */
    SwerveDriveTeleop swerveDriveTeleop = new SwerveDriveTeleop(drivetrain, xbox0, XboxController.Axis.kLeftY, XboxController.Axis.kLeftX, XboxController.Axis.kRightX);
    swerveDriveTeleop.setThrottle(0.4);
    drivetrain.setDefaultCommand(swerveDriveTeleop);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This command will run in autonomous
    try {
      return new HallAuton(drivetrain, "paths/output/" + "Test" + ".wpilib.json");
    } catch (java.io.IOException e) {
      e.printStackTrace();
      DriverStation.reportError("Could not load pathweaver swerve drive.", true);
      return null;
    }
  }
}
