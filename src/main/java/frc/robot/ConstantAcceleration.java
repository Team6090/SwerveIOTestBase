// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.geometry.SwerveVector;

public class ConstantAcceleration extends CommandBase {

  private SwerveDrive swerve;

  public ConstantAcceleration(SwerveDrive swerve) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(swerve);
    this.swerve = swerve;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    SwerveVector vector = new SwerveVector(0.5, 0, 0);
    swerve.drive(vector);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {}
}
