/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.gyro.NavXGyro;

public class Rotate extends CommandBase {

  private SwerveDrive swerve;
  private double degrees;
  private long duration;
  private double targetAngle;
  private NavXGyro gyro;
  /**
   * Creates a new Rotate.
   */
  public Rotate(SwerveDrive swerve, NavXGyro gyro, double degrees, long duration) {
    this.swerve = swerve;
    this.degrees = degrees;
    this.duration = duration;
    this.gyro = gyro;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = gyro.getContinuousAngle() + degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds vector = new ChassisSpeeds(0, 0, Math.toRadians(degrees) / (duration / 1000));
    swerve.drive(vector);
    System.out.println("[Rotate] TargetAngle: " + targetAngle + " Current Angle: " + gyro.getContinuousAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetAngle - gyro.getContinuousAngle()) < 8;
  }
}
