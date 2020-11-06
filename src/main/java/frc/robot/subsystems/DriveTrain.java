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
import net.bancino.robotics.swerveio.encoder.AnalogEncoder;
import net.bancino.robotics.swerveio.module.MK2SwerveModule;
import net.bancino.robotics.swerveio.pid.PIDController;
import net.bancino.robotics.swerveio.log.DashboardSwerveLogger;
import net.bancino.robotics.swerveio.geometry.Length;
import net.bancino.robotics.swerveio.geometry.Length.Unit;
import net.bancino.robotics.swerveio.geometry.ChassisDimension;
import net.bancino.robotics.swerveio.gyro.Gyro;

/**
 * The drivetrain subsystem drives the robot! (wow!).
 *
 * This subsystem consists of the following components:
 * - Swerve module (4x drive + pivot motor)
 *
 * This subsystem should provide the following functions:
 * - Run the drivetrain with joystick
 * - Run the drivetrain autonomously
 *
 * @author Jordan Bancino
 */
public class DriveTrain {

  public static SwerveDrive create(Gyro gyro) {
    return new SwerveDrive.Builder()
      .useDefaultKinematics(new ChassisDimension(new Length(29, Unit.INCHES)))
      .setGyro(gyro)
      .setModuleMap((map) -> {
        AnalogEncoder frontRightEncoder = new AnalogEncoder(Const.Encoder.FRONT_RIGHT_ANALOG_ENCODER, Const.Encoder.FRONT_RIGHT_ENCODER_OFFSET);
        AnalogEncoder frontLeftEncoder = new AnalogEncoder(Const.Encoder.FRONT_LEFT_ANALOG_ENCODER, Const.Encoder.FRONT_LEFT_ENCODER_OFFSET);
        AnalogEncoder rearLeftEncoder = new AnalogEncoder(Const.Encoder.REAR_LEFT_ANALOG_ENCODER, Const.Encoder.REAR_LEFT_ENCODER_OFFSET);
        AnalogEncoder rearRightEncoder = new AnalogEncoder(Const.Encoder.REAR_RIGHT_ANALOG_ENCODER, Const.Encoder.REAR_RIGHT_ENCODER_OFFSET);

        map.put(SwerveModule.Location.FRONT_RIGHT, new MK2SwerveModule(Const.CAN.FRONT_RIGHT_DRIVE_MOTOR, Const.CAN.FRONT_RIGHT_PIVOT_MOTOR, frontRightEncoder));
        map.put(SwerveModule.Location.FRONT_LEFT, new MK2SwerveModule(Const.CAN.FRONT_LEFT_DRIVE_MOTOR, Const.CAN.FRONT_LEFT_PIVOT_MOTOR, frontLeftEncoder));
        map.put(SwerveModule.Location.REAR_LEFT, new MK2SwerveModule(Const.CAN.REAR_LEFT_DRIVE_MOTOR, Const.CAN.REAR_LEFT_PIVOT_MOTOR, rearLeftEncoder));
        map.put(SwerveModule.Location.REAR_RIGHT, new MK2SwerveModule(Const.CAN.REAR_RIGHT_DRIVE_MOTOR, Const.CAN.REAR_RIGHT_PIVOT_MOTOR, rearRightEncoder));
      }, (module) -> {
        PIDController modulePid = module.getPivotPIDController();
        modulePid.setOutputRampRate(Const.PID.SWERVE_MODULE_RAMP_RATE);
        modulePid.setP(Const.PID.SWERVE_MODULE_P);
        modulePid.setI(Const.PID.SWERVE_MODULE_I);
        modulePid.setD(Const.PID.SWERVE_MODULE_D);
      })
      .build((swerve) -> {
        swerve.zeroDriveEncoders();
        //swerve.setFieldCentric(false);

        //swerve.setIdleAngle(0, false);

        swerve.startLogging(new DashboardSwerveLogger());
      });
  }
}
