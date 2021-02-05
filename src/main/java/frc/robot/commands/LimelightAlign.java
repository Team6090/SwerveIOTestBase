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
import net.bancino.robotics.swerveio.pid.MiniPID;
import net.bancino.robotics.swerveio.command.SwerveDriveTeleop;
import net.bancino.robotics.swerveio.command.SwerveDriveTeleopCommand;

@SuppressWarnings("unused")
public class LimelightAlign extends SwerveDriveTeleopCommand {

    /* Import parameters from the config file. Just slapped in here for SwerveIOTestBase. */
    private static final int rollingAverageWindow = 10;
    //private static final double rampRate = 0.5;
    private static final double maxOutput = 0.7;
    private static final double desiredDistancetoTarget = 204;
    private static final double desiredAngletoTarget = 0;
    private static final double desiredStrafetoTarget = 0;
    private static final double forwardP = 0.0018;
    private static final double forwardI = 0.0005;
    private static final double rotateP = 0.005;
    private static final double rotateI = 0.0001;
    private static final double rotateIZone = 15;
    private static final double strafeP = 0.008;
    private static final double strafeI = 0.001;

    private Limelight limelight;
    private boolean doFrontHatch;
    private boolean isFinished = false;
    private boolean isFieldCentric = false;
    private boolean fwdIsGood = false, strIsGood = false, rcwIsGood = false;

    /** Variables to hold distance and the vector speed on that axis. */
    private double fwd, str, rcw;

    /** Timeout variables */
    private long timeout = 5000;
    private long startTime = 0;

    /** Makes a new RollingAverage for every axis that uses the average limelight readings over so many readings. */
    private RollingAverage rollingAverageFWD = new RollingAverage(rollingAverageWindow);
    private RollingAverage rollingAverageRCW = new RollingAverage(rollingAverageWindow);
    private RollingAverage rollingAverageSTR = new RollingAverage(rollingAverageWindow);

    /** Make a PID controller for every axis that LimelightAlign corrects. P and I doubles defined in config file. */
    private MiniPID pidFWD = new MiniPID(forwardP, forwardI, 0);
    private MiniPID pidRCW = new MiniPID(rotateP, rotateI, 0);
    private MiniPID pidSTR = new MiniPID(strafeP, strafeI, 0);

    public LimelightAlign(SwerveDrive drivetrain, SwerveDriveTeleop swerveDriveTeleop, Limelight limelight, boolean doFrontHatch) {
        super(drivetrain, swerveDriveTeleop);
        this.limelight = limelight;
        this.doFrontHatch = doFrontHatch;
    }

    @Override
    public void initialize(SwerveDrive drivetrain) {
        /** Setting the setpoint that the PID will try to achieve for each axis. Defined in config file. */
        pidFWD.setSetpoint(desiredDistancetoTarget);
        pidRCW.setSetpoint(desiredAngletoTarget);
        pidSTR.setSetpoint(desiredStrafetoTarget);

        /** Setting the ramp rate for each axis. Defined in config file. */
        //pidFWD.setOutputRampRate(rampRate);
        //pidRCW.setOutputRampRate(rampRate);
        //pidSTR.setOutputRampRate(rampRate);

        /** Setting the maximum output value each axis. Defined in config file. */
        pidFWD.setOutputLimits(maxOutput);
        pidRCW.setOutputLimits(maxOutput);
        pidSTR.setOutputLimits(maxOutput);

        limelight.setPipeline(0);
        
        isFieldCentric = drivetrain.isFieldCentric();
        drivetrain.setFieldCentric(false);

        limelight.setLedMode(LedMode.FORCE_ON);
        /** For the front hatch, there's no need for head-on alignment where forward and strafe would matter. */
        if (doFrontHatch) {
            fwd = 0;
            str = 0;
        }
        startTime = System.currentTimeMillis();
        /** -1 is the value that means there is no timeout, so if you're not doing the front hatch, there needs to be a timeout. */
        if (doFrontHatch) {
            timeout = -1;
        }
    }

    @Override
    public void execute(SwerveDrive drivetrain, SwerveVector joystickVector) {

        /** If there's no target, no bueno. Exit the command. */
        isFinished = !limelight.hasValidTargets();

        /** 
         * This is really hard on the eyes, but these if statements determine if the past and current limelight
         * tx values are within bounds.
         */
        boolean firstScanInBounds = false;
        boolean currentScanInBounds = false;
        /** Covers the condition where tx exits the IZone bounds. */
        if (limelight.getHorizontalOffset() <= rotateIZone && limelight.getHorizontalOffset() >= -rotateIZone) {
            firstScanInBounds = true;
        } else {
            firstScanInBounds = false;
        }
        /** Covers the condition where tx enters the IZone bounds. */
        if (limelight.getHorizontalOffset() <= rotateIZone && limelight.getHorizontalOffset() >= -rotateIZone) {
            currentScanInBounds = true;
        } else {
            currentScanInBounds = false;
        }

        /** Uses the booleans determined above to set the RCW iZone. */
        if (!firstScanInBounds && currentScanInBounds) {
            pidRCW.reset();
            pidRCW.setI(rotateI);
        } else if (firstScanInBounds && !currentScanInBounds) {
            pidRCW.reset();
            pidRCW.setI(0);
        }

        /** 
         * RCW must be done for front and back hatches, so it gets calculated here before any other axis.
         * Values are put into the rolling average first to smooth out jumpy readings, and then put into the PID
         * loop for more efficient readings. (For all axis').
         */
        rollingAverageRCW.add(limelight.getHorizontalOffset());
        if (rollingAverageRCW.getCursor() == rollingAverageWindow - 1) {
            rcw = pidRCW.getOutput(rollingAverageRCW.get());
        } else {
            rcw = 0;
        }

        /**
         * Sets forward and strafe depending on whether or not you're doing the back port.
         * Forwards and strafe were set equal to zero in init.
         */
        if (!doFrontHatch) {
            double[] camtran = limelight.getCamTran();
            /** Forwards calculations. */
            rollingAverageFWD.add(camtran[2]);
            fwd = pidFWD.getOutput(rollingAverageFWD.get());
            /** Strafe calculations. */
            rollingAverageSTR.add(camtran[0]);
            str = pidSTR.getOutput(rollingAverageSTR.get());
        }

        /**
         * If you're not doing the front port, quit when all three values are
         * within the threshold (or when there is no target.) If you're only doing
         * the front port, end the program when the rotation is good.
         */
        if (!doFrontHatch) {
            isFinished = fwdIsGood && strIsGood && rcwIsGood;
        } else {
            isFinished = rcwIsGood;
        }

        /** If the timeout is hit, end the command. */
        if (timeout != -1 && System.currentTimeMillis() - startTime >= timeout) {
            isFinished = true;
        }

        /** Robot go vroom on the vector with swerve drive. */
        SwerveVector alignmentVector = new SwerveVector(fwd, str, -rcw);
        alignmentVector = alignmentVector.plus(joystickVector);
        drivetrain.drive(alignmentVector);
    }

    @Override
    public void end(SwerveDrive drivetrain, boolean interrupted) {
        limelight.setLedMode(LedMode.PIPELINE_CURRENT);
        drivetrain.setFieldCentric(isFieldCentric);
        rollingAverageRCW.reset();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
