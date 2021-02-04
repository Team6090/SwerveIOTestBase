/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.RollingAverage;
import frc.robot.Const;
import net.bancino.robotics.jlimelight.LedMode;
import net.bancino.robotics.jlimelight.Limelight;
import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.geometry.SwerveVector;
import net.bancino.robotics.swerveio.pid.MiniPID;

@SuppressWarnings("unused")
public class LimelightAlign extends CommandBase {

    /* Import parameters from the config file. Just slapped in here for SwerveIOTestBase. */
    private static final int rollingAverageWindow = 15;
    //private static final double rampRate = 0.5;
    private static final double maxOutput = 0.7;
    private static final double desiredDistancetoTarget = 204;
    private static final double desiredAngletoTarget = 0;
    private static final double desiredStrafetoTarget = 0;
    private static final double forwardP = 0.001;
    private static final double forwardI = 0.001;
    private static final double rotateP = 0.005;
    private static final double rotateI = 0.0001;
    private static final double rotateIZone = 15;
    private static final double strafeP = 0.008;
    private static final double strafeI = 0.001;

    private SwerveDrive drivetrain;
    private Limelight limelight;
    private boolean doFrontHatch;
    private boolean isFinished = false;
    private boolean isFieldCentric = false;
    private boolean fwdIsGood = false, strIsGood = false, rcwIsGood = false;

    /** Variables to hold distance and the vector speed on that axis. */
    private double fwd, str, rcw;
    private XboxController.Axis xboxFWD, xboxSTR, xboxRCW;

    /** Timeout variables */
    private long timeout = 5000;
    private long startTime = 0;
    
    /** Teleop variables */
    private double deadband, throttle;
    private XboxController joystick;

    /** Makes a new RollingAverage for every axis that uses the average limelight readings over so many readings. */
    private RollingAverage rollingAverageFWD = new RollingAverage(rollingAverageWindow);
    private RollingAverage rollingAverageRCW = new RollingAverage(rollingAverageWindow);
    private RollingAverage rollingAverageSTR = new RollingAverage(rollingAverageWindow);

    /** Make a PID controller for every axis that LimelightAlign corrects. P and I doubles defined in config file. */
    private MiniPID pidFWD = new MiniPID(forwardP, forwardI, 0);
    private MiniPID pidRCW = new MiniPID(rotateP, rotateI, 0);
    private MiniPID pidSTR = new MiniPID(strafeP, strafeI, 0);

    public LimelightAlign(SwerveDrive drivetrain, Limelight limelight, XboxController joystick, XboxController.Axis xboxFWD, XboxController.Axis xboxSTR, XboxController.Axis xboxRCW, boolean doFrontHatch) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.joystick = joystick;
        this.xboxFWD = xboxFWD;
        this.xboxSTR = xboxSTR;
        this.xboxRCW = xboxRCW;
        this.doFrontHatch = doFrontHatch;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
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
    public void execute() {

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

        /** Adds in the deadbanding for manual Xbox control. */
        setThrottle(0.9);
        setThrottle(0.2);
        fwd += -throttle(deadband(joystick.getRawAxis(xboxFWD.value)));
        str += throttle(deadband(joystick.getRawAxis(xboxSTR.value)));
        rcw += throttle(deadband(joystick.getRawAxis(xboxRCW.value)));

        /** Robot go vroom on the vector with swerve drive. */
        SwerveVector alignmentVector = new SwerveVector(fwd, str, -rcw);
        drivetrain.drive(alignmentVector);
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setLedMode(LedMode.PIPELINE_CURRENT);
        drivetrain.setFieldCentric(isFieldCentric);
        rollingAverageRCW.reset();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    /**
     * Set a deadband on the joystick. This is a little bit of range around the zero
     * mark that does absolutely nothing. This is helpful for joysticks that are
     * overly sensitive or don't always read zero in the neutral position.
     * 
     * @param deadband The deadband to set, between 0 and 1.
     */
    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    /**
     * Set a throttle on the joystick. This is helpful for limiting the top speed of
     * the swerve drive, which can be useful for training.
     * 
     * @param throttle The throttle to set. This is the maximum speed the joystick
     *                 should output. So, to throttle this command at 50% power,
     *                 you'd put in 0.5 for the throttle.
     */
    public void setThrottle(double throttle) {
        this.throttle = throttle;
    }

    /**
     * Calculate a deadband
     * 
     * @param raw The input on the joystick to mod
     * @return The result of the mod.
     */
    protected double deadband(double raw) {
        /* Compute the deadband mod */
        if (raw < 0.0d) {
            if (raw <= -deadband) {
                return (raw + deadband) / (1 - deadband);
            } else {
                return 0.0d;
            }
        } else {
            if (raw >= deadband) {
                return (raw - deadband) / (1 - deadband);
            } else {
                return 0.0d;
            }
        }
    }

    /**
     * Throttle a raw value based on the currently set throttle.
     * 
     * @param raw The raw joystick value to scale.
     * @return A scaled joystick value.
     */
    protected double throttle(double raw) {
        return raw * throttle;
    }

}
