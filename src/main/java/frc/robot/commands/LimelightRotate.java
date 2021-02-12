/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.RollingAverage;
import frc.robot.Const;
import net.bancino.robotics.jlimelight.LedMode;
import net.bancino.robotics.jlimelight.Limelight;
import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.geometry.SwerveVector;
import net.bancino.robotics.swerveio.pid.DefaultPIDController;
import net.bancino.robotics.swerveio.command.SwerveDriveTeleop;
import net.bancino.robotics.swerveio.command.SwerveDriveTeleopCommand;

@SuppressWarnings("unused")
public class LimelightRotate extends SwerveDriveTeleopCommand {

    /* Import parameters from the config file. Just slapped in here for SwerveIOTestBase. */
    private static final int rollingAverageWindow = 10;
    //private static final double rampRate = 0.5;
    private static final double maxOutput = 0.7;
    private static final double desiredAngletoTarget = 0;


    /** PID long distance slot (RCW) */
    

    /** PID short/medium distance slot (RCW) */
    private static final double rotateP_S0 = 0.00523;
    private static final double rotateI_S0 = 0.000063;
    private static final double rotateIZone_S0 = 4;

    private Limelight limelight;
    private boolean doFrontHatch;
    private boolean isFinished;
    private boolean isFieldCentric = false;

    /** Variable to hold the value for rotation. */
    private double rcw;

    /** Time variables */
    private long startTime = 0;

    /** Makes a new RollingAverage. */
    private RollingAverage rollingAverageRCW = new RollingAverage(rollingAverageWindow);

    /** Make a PID controller for rotation. */
    private DefaultPIDController pidRCW = new DefaultPIDController(rotateP_S0, rotateI_S0, 0);

    public LimelightRotate(SwerveDrive drivetrain, SwerveDriveTeleop swerveDriveTeleop, Limelight limelight) {
        super(drivetrain, swerveDriveTeleop);
        this.limelight = limelight;
    }

    @Override
    public void initialize(SwerveDrive drivetrain) {

        /** Setting the maximum output value each axis. Defined in config file. */
        pidRCW.setOutputLimits(maxOutput);

        limelight.setPipeline(0);
        
        isFieldCentric = drivetrain.isFieldCentric();
        drivetrain.setFieldCentric(false);

        limelight.setLedMode(LedMode.FORCE_ON);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(SwerveDrive drivetrain, SwerveVector joystickVector) {

        /** If there's no target, no bueno. Exit the command. Also, wait 1 second to give the Limelight a chance to turn on. */
        if (!limelight.hasValidTargets() && System.currentTimeMillis() - startTime > 1000) {
            isFinished = true;
            return;
        }

        /** 
         * This is really hard on the eyes, but these if statements determine if the past and current limelight
         * tx values are within bounds.
         */
        boolean firstScanInBounds = false;
        boolean currentScanInBounds = false;
        /** Covers the condition where tx exits the IZone bounds. */
        if (limelight.getHorizontalOffset() <= rotateIZone_S0 && limelight.getHorizontalOffset() >= -rotateIZone_S0) {
            firstScanInBounds = true;
        } else {
            firstScanInBounds = false;
        }
        /** Covers the condition where tx enters the IZone bounds. */
        if (limelight.getHorizontalOffset() <= rotateIZone_S0 && limelight.getHorizontalOffset() >= -rotateIZone_S0) {
            currentScanInBounds = true;
        } else {
            currentScanInBounds = false;
        }

        /** Uses the booleans determined above to set the RCW IZone. */
        if (!firstScanInBounds && currentScanInBounds) {
            pidRCW.reset();
            pidRCW.setI(0, rotateI_S0);
        } else if (firstScanInBounds && !currentScanInBounds) {
            pidRCW.reset();
            pidRCW.setI(0, rotateI_S0);
        }

        /** 
         * Values are put into the rolling average first to smooth out jumpy readings, and then put into the PID
         * loop for more efficient readings.
         */
        rollingAverageRCW.add(limelight.getHorizontalOffset());
        if (rollingAverageRCW.getCursor() == rollingAverageWindow - 1) {
            rcw = pidRCW.getOutput(rollingAverageRCW.get(), desiredAngletoTarget);
        } else {
            rcw = 0;
        }

        /** Robot go vroom on the vector with swerve drive. */
        SwerveVector alignmentVector = new SwerveVector(0, 0, -rcw);
        alignmentVector = alignmentVector.plus(joystickVector);
        drivetrain.drive(alignmentVector);
    }

    @Override
    public void end(SwerveDrive drivetrain, boolean interrupted) {
        limelight.setLedMode(LedMode.PIPELINE_CURRENT);
        drivetrain.setFieldCentric(isFieldCentric);
        rollingAverageRCW.reset();
        pidRCW.reset();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}

