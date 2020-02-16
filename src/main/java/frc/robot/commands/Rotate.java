/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.SwerveVector;

public class Rotate extends CommandBase {

  private SwerveDrive swerve;
  private long startTime, currentTime;
  private double currentAngle, targetAngle;

  /**
   * Creates a new PointToZero.
   */
  public Rotate(SwerveDrive swerve) {
    addRequirements(swerve);
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTime = System.currentTimeMillis();
    startTime = currentTime;
    currentAngle = swerve.getLastGyroAngle();
    targetAngle = currentAngle + 90;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(new ChassisSpeeds(0, 0, Math.toRadians(45)));
    currentTime = System.currentTimeMillis(); 
    currentAngle = swerve.getLastGyroAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    targetAngle = currentAngle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return currentTime - startTime > 2000;
    return Math.abs(currentAngle - targetAngle) <= 20;
  }
}
