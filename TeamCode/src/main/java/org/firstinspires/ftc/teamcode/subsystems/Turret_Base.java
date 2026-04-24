package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.MathUtils;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.HydraPIDFController;
import org.firstinspires.ftc.teamcode.objects.LowPassFilter;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.VisionResult;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.TurretTrackMode;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class Turret_Base implements Subsystem {
    protected final HydraOpMode mOp;
    protected final VoltageSensor voltageSensor;
    protected final AbsoluteAnalogEncoder AnalogTurretEncoder;
    protected Motor.Encoder TurretEncoder;
    public static double mPosChangeRateDegrees = 40;
    protected double TurretSyncOffset = 0.0;
    protected long lastVisionTimestamp;
    protected boolean autoSetAction;
    protected double autoSetAngle;
    protected boolean visionLocked;
    protected final Debouncer disableOdoTrackWithCircle;
    protected final Debouncer disableVisionTrackWithCircle;
    protected final Debouncer triangleDebounce;
    protected final Imu imu;
    protected double UserInput = 0;
    public static int VisionRefreshTimeMs = 0; //seems to be better with zero.
    protected final Vector2d goalPosition;
    public static double manualAngle = 0;
    public static boolean manualAngleEnable = false;
    protected TurretTrackMode trackMode;
    protected final TurretTrackMode initialTrackMode;
    public static double userInputExponent = 2;
    public static double TurretRobotAngVelThresh = 8; // degrees/s
    public static double TurretRobotVelThresh = 1000.0; // inches/s
    private double visionSetpoint = 0.0;
    private double odometrySetpoint = 0.0;
    public HydraPIDFController TurretAngleController;
    public static double default_P = 0.98, default_I = 0, default_D = 0, default_F = 0;
    public static double vision_P = 0.94, vision_I = 0.3, vision_D = 0, vision_F = 0;
    public static double odometry_P = 0.98, odometry_I = 0, odometry_D = 0, odometry_F = 0;
    public static double turretAngleCntlrILimit = 3;
    private boolean visionUsedLast = false;
    public static double turretVisionLockVelThresh = 6; // Degrees/s
    public static double turretVisionSetPointVelThresh = 10; // Degrees/s
    public LowPassFilter FiltVisionSP;
    private static double visionFiltGain = 0.4;


    public Turret_Base(HydraOpMode opMode, Imu imu, VisionMode target, TurretTrackMode trackingMode, boolean flipEncoder) {
        mOp = opMode;
        voltageSensor = mOp.mHardwareMap.get(VoltageSensor.class, "Control Hub");
        // Adjust target location for alliance
        if (target == VisionMode.VisionMode_BlueGoal) {
            goalPosition = new Vector2d(-59, -55.189); // Per CAD Tag is located at 59, -55.189 Back of goal -72, -72
        }
        else {
            goalPosition = new Vector2d(-59, 55.189);
        }
        AnalogTurretEncoder = new AbsoluteAnalogEncoder(mOp.mHardwareMap,"TurretServoFb", Constants.TurretServoAnalogRangeVolts, AngleUnit.DEGREES)
                .zero(Constants.TurretEncoderOffset)
                //.setReversed(true)
        ;
        TurretEncoder = new Motor(mOp.mHardwareMap, "leftBack").encoder
                .overrideResetPos((int) TurretSyncOffset)
        ;
        // Fix issue caused by Road Runner having different encoder direction than HyDrive.
        if (flipEncoder) {
            TurretEncoder.setDirection(Motor.Direction.REVERSE);
        } else {
            TurretEncoder.setDirection(Motor.Direction.FORWARD);
        }
        lastVisionTimestamp = 0;
        autoSetAction = false;
        autoSetAngle = 0;
        visionLocked = false;
        initialTrackMode = trackingMode;
        this.trackMode = trackingMode;
        disableOdoTrackWithCircle = new Debouncer(Constants.debounceLong);
        disableVisionTrackWithCircle = new Debouncer(Constants.debounceLong * 10);
        triangleDebounce = new Debouncer(1);
        this.imu = imu;
        TurretAngleController = new HydraPIDFController(default_P, default_I, default_D, default_F);
        TurretAngleController.setTolerance(0.5);
        TurretAngleController.setIntegrationBounds(-turretAngleCntlrILimit,turretAngleCntlrILimit);
        FiltVisionSP = new LowPassFilter(visionFiltGain);
    }

    @Override
    public void HandleUserInput() {
        UserInput = mOp.mOperatorGamepad.right_stick_x;
        // UserInput = Range.scale(UserInput,0,1,Constants.TurretMinAngle, Constants.TurretMaxAngle); // Scale stick to Min Max Turret Angle
        disableOdoTrackWithCircle.In(mOp.mOperatorGamepad.circle);
        if (disableOdoTrackWithCircle.Out()) {
            disableOdoTrackWithCircle.Used();
            if (trackMode == TurretTrackMode.OdoAndVision) {
                trackMode = TurretTrackMode.VisionOnly;
            } else {
                trackMode = TurretTrackMode.OdoAndVision;
            }
            mOp.mOperatorGamepad.rumbleBlips(1);
        }
        disableVisionTrackWithCircle.In(mOp.mOperatorGamepad.circle);
        if (disableVisionTrackWithCircle.Out()) {
            disableVisionTrackWithCircle.Used();
            if (trackMode == TurretTrackMode.Disabled) {
                trackMode = TurretTrackMode.OdoAndVision;
            } else {
                trackMode = TurretTrackMode.Disabled;
            }
            mOp.mOperatorGamepad.rumbleBlips(2);
        }
        triangleDebounce.In(mOp.mOperatorGamepad.triangle);
        if (triangleDebounce.Out()) {
            triangleDebounce.Used();
            GoHome();
        }
    }

    @Override
    public void Process() {
        VisionResult vision = null;
        if (mOp.mVision != null) {
            vision = mOp.mVision.GetResult();
        }
        Pose2d currentPose = null;
        if (imu != null) {
            currentPose = imu.GetPose();
//            double Pose_x
//                    = currentPose.position.x;
//            double Pose_y = currentPose.position.y;
            double Pose_heading
                    = Math.toDegrees(currentPose.heading.toDouble());
            //mOp.mTelemetry.addData("Pose x", Pose_x); // Inches
            //mOp.mTelemetry.addData("Pose y", Pose_y); // Inches
            mOp.mTelemetry.addData("Heading", Pose_heading);
        }
        double servoFbPosition = TurretEncoder.getPosition() * Constants.TurretDegreesPerTick; //Degrees
        double TurretAnalogPositon = MathUtils.normalizeDegrees(AnalogTurretEncoder.getCurrentPosition(), false)/Constants.TurretGearRatioTurretToServo; //Degrees
        mOp.mTelemetry.addData("TurretServoFb", servoFbPosition);
        //mOp.mTelemetry.addData("TurretAnalogPos", TurretAnalogPositon);

        // Attempt to get continuous vision and Odometry setpoints
        if (vision != null) {
            visionSetpoint = servoFbPosition + vision.GetXOffset();
            // visionSetpoint = Prev_servoFbPosition + vision.GetXOffset(); // May need to use previous position due to Vision lag
            //mOp.mTelemetry.addData("VisionOffset", vision.GetXOffset());
            mOp.mTelemetry.addData("VisionSetpoint", visionSetpoint);
        }
        if (currentPose != null) {
            // angle from the robot's current x,y position to the goal
            double angleToGoal = AutoTangent(currentPose.position, goalPosition);
            // get the robot's heading, offset by 180 because the turret is on the back
            double robotHeading = Math.toDegrees(currentPose.heading.toDouble()) - 180;
            // calculate the angle of the turret to point at the goal
            odometrySetpoint = MathUtils.normalizeDegrees(robotHeading - angleToGoal, false); // removed Clamp so we can see what the odometery really wants
            mOp.mTelemetry.addData("OdometrySetpoint", odometrySetpoint);
        }
        //mOp.mTelemetry.addData("AutoSetAngle", autoSetAngle);

        // Order of priorities
        // 1) Auto sets a desired position
        // 2) if a tag is visible, track it
        // 3) turn the turret towards the target using odometry
        // 4) take user input
        if (autoSetAction) {
            SetAngleController(autoSetAngle, default_P, default_I, default_D, default_F);
            TurretAngleController.clearTotalError(); // Reset Integrator
            visionUsedLast = false;

            // Use Vision Tracking if enabled, valid and Robot is moving slow.
        } else if (VisionTrackingEnabled() && vision != null && vision.isValid() && RobotVelocityOK()){
            //mOp.mTelemetry.addData("AprilTag", vision.GetTagClass());
            //mOp.mTelemetry.addData("AprilTag", vision.GetXOffset());
            //mOp.mTelemetry.addData("AprilTag", vision.GetYOffset());
            if ((vision.GetTimestamp() > lastVisionTimestamp + VisionRefreshTimeMs)){ //&& RobotVelocityOK() && TurretVelocityOK(turretVisionSetPointVelThresh)) {
                //mOp.mTelemetry.addData("VisionLatency", vision.GetLatency()); //Checking latency.
                //mOp.mTelemetry.addData("Visiontimestamp", vision.GetTimestamp());
                //double rotate = vision.GetXOffset();
                //mOp.mTelemetry.addData("rotate", rotate);
                lastVisionTimestamp = vision.GetTimestamp();
                if (!visionUsedLast) {
                    TurretAngleController.clearTotalError();
                    FiltVisionSP.reset();
                    FiltVisionSP.update(servoFbPosition, visionFiltGain); //Seed the Filter with the current position
                }
                FiltVisionSP.update(visionSetpoint,visionFiltGain);
                mOp.mTelemetry.addData("FiltVisionSP", FiltVisionSP.getValue());
                visionSetpoint = FiltVisionSP.getValue(); //ToDo: evaluate filter performance before using.
                SetAngleController(visionSetpoint, vision_P, vision_I, vision_D, vision_F);
                visionUsedLast = true;
            }
        } else if (OdometryTrackingEnabled() && currentPose != null) {
            if (visionUsedLast) {
                TurretAngleController.clearTotalError(); // Reset Integrator
            }
            SetAngleController(odometrySetpoint, odometry_P, odometry_I, odometry_D, odometry_F);
            visionUsedLast = false;

        } else if (manualAngleEnable) {
            SetAngleController(manualAngle, default_P, default_I, default_D, default_F);
            visionUsedLast = false;

        } else {
            // scale user input
            double userInputMagnitude = Math.abs(UserInput);
            double userInputSign = Math.signum(UserInput);
            double userInputScalar = userInputSign * Math.pow(userInputMagnitude, userInputExponent);
            double position_change = userInputScalar * mPosChangeRateDegrees;
            if (position_change != 0) {
                // get the last set position and calculate the new position
                SetAngleController(servoFbPosition + position_change, default_P, default_I, default_D, default_F);
                visionUsedLast = false;
            }
        }

        // Don't let the Integrator wind-up
        if(servoFbPosition <= Constants.TurretMinAngle || servoFbPosition >= Constants.TurretMaxAngle ){
            TurretAngleController.clearTotalError();
        }
        double turretAdjustment = TurretAngleController.calculate(servoFbPosition);
        // Command Turret Servo
        SetTurretAngle(servoFbPosition + turretAdjustment);

        // TODO: *Review* might need fancier logic for this
        visionLocked = Math.abs(visionSetpoint - servoFbPosition) < Constants.TurretDeadbandDegrees && TurretVelocityOK(turretVisionLockVelThresh) && visionUsedLast;

        // Clear vision locked
        if (!VisionTrackingEnabled() || System.currentTimeMillis() - lastVisionTimestamp > Constants.TurretVisionLockTimeoutMs) {
            visionLocked = false;
        }

        //mOp.mTelemetry.addData("TurretCntrlrIterm", TurretAngleController.getITerm());
        mOp.mTelemetry.addData("TurretAdjustment", turretAdjustment);
        mOp.mTelemetry.addData("VisionUsedLast",visionUsedLast);
        //mOp.mTelemetry.addData("AutoPos", autoSetPos);
        mOp.mTelemetry.addData("VisionLocked", visionLocked);
        mOp.mTelemetry.addData("AutoTrack", trackMode);
        //double Prev_servoFbPosition = servoFbPosition;
    }

    public boolean Locked() {
        return visionLocked;
    }

    public void ForceUnlock(){
        visionLocked = false;
        // reset the timestamp so we can use the very next update from the camera
        lastVisionTimestamp += 50;
        TurretAngleController.clearTotalError();
    }

    protected static double AutoTangent(Vector2d start, Vector2d end) {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public void GoHome() {
        autoSetAction = true;
        autoSetAngle = 0;
        Process();
        autoSetAction = false;
    }

    private double Clamp(double position) {
        return Range.clip(position, Constants.TurretMinAngle, Constants.TurretMaxAngle);
    }

    protected abstract void SetTurretAngle(double angle);
    protected abstract double getPosition();

    protected boolean OdometryTrackingEnabled() {
        return trackMode == TurretTrackMode.OdoAndVision;
    }

    protected boolean VisionTrackingEnabled() {
        return trackMode != TurretTrackMode.Disabled;
//        switch (autoTrack) {
//            case autoTrackDisabled:
//                return false;
//            case autoTrackEnabled:
//            case autoTrackOdoDisabled:
//            default:
//                return true;
//        }
    }
    //Check if moving too fast to use vision
    protected boolean RobotVelocityOK(){
        if (imu == null) {
            return true;
        }
        PoseVelocity2d velocity = imu.GetPoseVelocity();
        // Check if linear velocity is less than thresh inches/s and angular is less than thresh degrees/s
        //mOp.mTelemetry.addData("RobotLinearVelocity", velocity.linearVel.norm());
        mOp.mTelemetry.addData("RobotAngularVelocity", Math.toDegrees(velocity.angVel));
        return velocity.linearVel.norm() < TurretRobotVelThresh && Math.abs(velocity.angVel) < Math.toRadians(TurretRobotAngVelThresh);
    }

    /**
     *
     * @return false if the turret is moving too fast
     */
    protected boolean TurretVelocityOK(double threshold){
        mOp.mTelemetry.addData("TurretVelocity", TurretEncoder.getCorrectedVelocity() * Constants.TurretDegreesPerTick);
        return (Math.abs(TurretEncoder.getCorrectedVelocity() * Constants.TurretDegreesPerTick) < threshold);
    }

    public void SetAngleController(double setPoint,double P, double I, double D, double F) {
        TurretAngleController.setPIDF(P, I, D, F);
        TurretAngleController.setSetPoint(Clamp(setPoint));
    }


    /*
     * ROAD RUNNER API
     */
    /**
     * Get a new action object for Road Runner to run
     * @param action: the action to run in this instance
     * @return the action object for RR to use
     */
    public Action GetLockAction() {
        return new Turret_Base.RunLockAction();
    }

    public Action GetDisableAction(boolean disable) {
        return new Turret.RunDisableAction(disable);
    }
    /**
     * Runs the supplied action until completion
     */
    public class RunLockAction implements Action {
        // run has been called once
        protected boolean started = false;
        protected ElapsedTime timer;
        public RunLockAction() {
            timer = new ElapsedTime();
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {
                timer.reset();
                started = true;
                ForceUnlock();
            }
            return !visionLocked && timer.milliseconds() < Constants.TurretVisionLockTimeoutMs;
        }
    }

    public class RunDisableAction implements Action {
        boolean disable;

        public RunDisableAction(boolean disable) {
            this.disable = disable;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (disable) {
                trackMode = TurretTrackMode.Disabled;
            } else {
                trackMode = initialTrackMode;
                ForceUnlock();
            }
            return false;
        }
    }

    public Action GetSetAction(double position) {
        return new Turret.RunSetAction(position);
    }

    public class RunSetAction implements Action {
        private boolean started = false;
        private final double angleToSet;
        public RunSetAction(double angleToSet) {
            this.angleToSet = angleToSet;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                started = true;
                autoSetAction = true;
                autoSetAngle = angleToSet;
            }
            Process();
            autoSetAction = false;
            /*if (visionLocked) {
                autoSetAction = false;
                return false;
            } else
            if (Math.abs(GetPositionFromFb() - autoSetPos) < CalcPositionOffsetAngle(Constants.TurretDeadbandDegrees)) {
                autoSetAction = false;
                return false;
            }*/
            return false;
        }
    }
}
