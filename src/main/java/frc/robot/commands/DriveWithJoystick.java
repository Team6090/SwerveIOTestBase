/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Const;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.SwerveVector;
import net.bancino.robotics.swerveio.pid.AbstractPIDController;
import net.bancino.robotics.swerveio.module.AbstractSwerveModule;
import net.bancino.robotics.swerveio.gyro.AbstractGyro;

import edu.wpi.first.wpilibj.GenericHID;

public class DriveWithJoystick extends CommandBase {

  private final double MIN_THROTTLE = 0.5;
  private final double MAX_THROTTLE = 1;
  private final double DEADBAND = 0.3;

  private XboxController xbox;
  private XboxController.Axis forwardAxis, strafeAxis, angularAxis;
  private SwerveDrive swerve;
  private AbstractGyro gyro;

  /**
   * Creates a new DriveWithJoystick.
   */
  public DriveWithJoystick(SwerveDrive swerve, AbstractGyro gyro, XboxController xbox, XboxController.Axis forwardAxis, XboxController.Axis strafeAxis, XboxController.Axis angularAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    this.xbox = xbox;
    this.forwardAxis = forwardAxis;
    this.strafeAxis = strafeAxis;
    this.angularAxis = angularAxis;
    this.swerve = swerve;
    this.gyro = gyro;

    SmartDashboard.putNumber("Joystick/PID/P", Const.PID.SWERVE_MODULE_P);
    SmartDashboard.putNumber("Joystick/PID/I", Const.PID.SWERVE_MODULE_I);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
  }

  private int lastAngle = 0;
  private boolean pivotPosOnly = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (AbstractSwerveModule module : swerve.getModuleMap().values()) {
      AbstractPIDController pid = module.getPivotPIDController();
      pid.setP(SmartDashboard.getNumber("Joystick/PID/P", 0));
      pid.setI(SmartDashboard.getNumber("Joystick/PID/I", 0));
    }

    SmartDashboard.putNumber("Joystick/XBoxVertical (Raw)", xBoxLeftJoystickVertical());

    double fwd = -throttle(deadband(xbox.getRawAxis(forwardAxis.value))) * 0.4;
    double str = throttle(deadband(xbox.getRawAxis(strafeAxis.value))) * 0.4;
    double rcw = throttle(deadband(xbox.getRawAxis(angularAxis.value)) * 0.4);

    if (xbox.getBumper(GenericHID.Hand.kLeft)) {
      gyro.zero();
    }

    swerve.drive(new SwerveVector(fwd, str, rcw));

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

  private double throttle(double raw) {
    //double throttle = SmartDashboard.getNumber("DB/Slider 0", MAX_THROTTLE);
    double throttle = MAX_THROTTLE;
    if (throttle > MAX_THROTTLE)
      throttle = MAX_THROTTLE;
    else if (throttle < MIN_THROTTLE)
      throttle = MIN_THROTTLE;
    return raw * throttle;
  }
  /**
   * Calculate a deadband
   * 
   * @param raw The input on the joystick to mod
   * @return The result of the mod.
   */
  private double deadband(double raw) {
    /* This will be our result */
    double mod;
    /* Compute the deadband mod */
    if (raw < 0.0d) {
      if (raw <= -DEADBAND) {
        mod = (raw + DEADBAND) / (1 - DEADBAND);
      } else {
        mod = 0.0d;
      }
    } else {
      if (raw >= DEADBAND) {
        mod = (raw - DEADBAND) / (1 - DEADBAND);
      } else {
        mod = 0.0d;
      }
    }
    /* Return the result. */
    return mod;
  }

  /**
   * Get the horizontal raw value of the left joystick on the XBox controller.
   * 
   * @return The current position of the left joystick in the horizontal
   *         direction.
   */
  public double xBoxLeftJoystickHorizontal() {
    return xbox.getRawAxis(0);
  }

  /**
   * Get the vertical raw value of the left joystick on the XBox controller.
   * 
   * @return The current position of the left joystick in the vertical direction.
   */
  public double xBoxLeftJoystickVertical() {
    return xbox.getRawAxis(1);
  }

  /**
   * Get the horizontal raw value of the right joystick on the XBox controller.
   * 
   * @return The current position of the right joystick in the horizontal
   *         direction.
   */
  public double xBoxRightJoystickHorizontal() {
    return xbox.getRawAxis(4);
  }

  /**
   * Get the vertical raw value of the right joystick on the XBox controller.
   * 
   * @return The current position of the right joystick in the vertical direction.
   */
  public double xBoxRightJoystickVertical() {
    return xbox.getRawAxis(5);
  }
}
