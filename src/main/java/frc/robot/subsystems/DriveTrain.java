/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Const;

import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.module.SwerveModule;
import net.bancino.robotics.swerveio.module.MK3SwerveModule;
import net.bancino.robotics.swerveio.pid.PIDController;
import net.bancino.robotics.swerveio.log.DashboardSwerveLogger;
import net.bancino.robotics.swerveio.geometry.Length;
import net.bancino.robotics.swerveio.geometry.Length.Unit;
import net.bancino.robotics.swerveio.geometry.ChassisDimension;
import net.bancino.robotics.swerveio.gyro.Gyro;

/**
 * The drivetrain subsystem drives the robot!
 *
 * This subsystem consists of the following components:
 * - Swerve module (4x drive + pivot motor)
 *
 * This subsystem should provide an interface for the
 * following functions:
 * 
 * - Running the drivetrain with joystick
 * - Running the drivetrain autonomously
 *
 * @author Jordan Bancino
 */
public class DriveTrain {

  public static SwerveDrive create(Gyro gyro) {
    return new SwerveDrive.Builder()
      .useDefaultKinematics(new ChassisDimension(new Length(22.5, Unit.INCHES)))
      // TODO: Add this back for Gyro tuning
      //.setGyro(gyro)
      //.setAnglePID(Const.PID.ANGLE_STANDING_SLOT, Const.PID.ANGLE_MOVING_SLOT, (pid) -> {
      //  pid.setP(Const.PID.ANGLE_STANDING_SLOT, Const.PID.SWERVE_DRIVE_ANGLE_STANDING_P);
      //  pid.setI(Const.PID.ANGLE_STANDING_SLOT, Const.PID.SWERVE_DRIVE_ANGLE_STANDING_I);
      //  pid.setD(Const.PID.ANGLE_STANDING_SLOT, Const.PID.SWERVE_DRIVE_ANGLE_STANDING_D);
      //
      //  pid.setP(Const.PID.ANGLE_MOVING_SLOT, Const.PID.SWERVE_DRIVE_ANGLE_MOVING_P);
      //  pid.setI(Const.PID.ANGLE_MOVING_SLOT, Const.PID.SWERVE_DRIVE_ANGLE_MOVING_I);
      //  pid.setD(Const.PID.ANGLE_MOVING_SLOT, Const.PID.SWERVE_DRIVE_ANGLE_MOVING_D);
      //  
      //  pid.setAcceptableError(Const.PID.SWERVE_DRIVE_ACCEPTABLE_ERROR);
      //})
      .setRampRate(Const.PID.SWERVE_MODULE_RAMP_RATE)
      .setModuleMap((map) -> {
        map.put(SwerveModule.Location.FRONT_RIGHT, new MK3SwerveModule(Const.CAN.FRONT_RIGHT_DRIVE_MOTOR, Const.CAN.FRONT_RIGHT_PIVOT_MOTOR, Const.CAN.FRONT_RIGHT_ENCODER, Const.Angle.FRONT_RIGHT_OFFSET));
        map.put(SwerveModule.Location.FRONT_LEFT, new MK3SwerveModule(Const.CAN.FRONT_LEFT_DRIVE_MOTOR, Const.CAN.FRONT_LEFT_PIVOT_MOTOR, Const.CAN.FRONT_LEFT_ENCODER, Const.Angle.FRONT_LEFT_OFFSET));
        map.put(SwerveModule.Location.REAR_LEFT, new MK3SwerveModule(Const.CAN.REAR_LEFT_DRIVE_MOTOR, Const.CAN.REAR_LEFT_PIVOT_MOTOR, Const.CAN.REAR_LEFT_ENCODER, Const.Angle.REAR_LEFT_OFFSET));
        map.put(SwerveModule.Location.REAR_RIGHT, new MK3SwerveModule(Const.CAN.REAR_RIGHT_DRIVE_MOTOR, Const.CAN.REAR_RIGHT_PIVOT_MOTOR, Const.CAN.REAR_RIGHT_ENCODER, Const.Angle.REAR_RIGHT_OFFSET));
      }, (module) -> {
        PIDController modulePid = module.getPivotPIDController();
        //modulePid.setOutputRampRate(Const.PID.SWERVE_MODULE_RAMP_RATE);
        modulePid.setP(Const.PID.SWERVE_MODULE_P);
        modulePid.setI(Const.PID.SWERVE_MODULE_I);
        modulePid.setD(Const.PID.SWERVE_MODULE_D);
      })
      .build((swerve) -> {
        //swerve.setFieldCentric(false);

        //swerve.setIdleAngle(0, false);

        swerve.startLogging(new DashboardSwerveLogger());
      });
  }
}
