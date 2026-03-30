package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.VisionResult;
import org.firstinspires.ftc.teamcode.types.Constants;
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
    public static int VisionRefreshTimeMs = 10;
    protected final Vector2d goalPosition;
    private boolean first;
    private double firstUpdate;
    public static double manualAngle = 0;
    public static boolean manualAngleEnable = false;
    public enum autoTrackType { autoTrackEnabled, autoTrackOdoDisabled, autoTrackDisabled };
    public static autoTrackType autoTrack;
    public static double userInputExponent = 2;

    public Turret_Base(HydraOpMode opMode, Imu imu, VisionMode target) {
        mOp = opMode;
        voltageSensor = mOp.mHardwareMap.get(VoltageSensor.class, "Control Hub");
        // Adjust target location for alliance
        if (target == VisionMode.VisionMode_BlueGoal) {
            goalPosition = new Vector2d(-72, -72);
        }
        else {
            goalPosition = new Vector2d(-72, 72);
        }
        AnalogTurretEncoder = new AbsoluteAnalogEncoder(mOp.mHardwareMap,"TurretServoFb", Constants.TurretServoAnalogRangeVolts, AngleUnit.DEGREES)
                .zero(Constants.TurretEncoderOffset)
                //.setReversed(true)
        ;
        TurretEncoder = new Motor(mOp.mHardwareMap, "leftBack").encoder
                .setDirection(Motor.Direction.REVERSE) // ToDo: *Setup*: Set based on Encoder orientation and Positive rotation convention
                .overrideResetPos((int) TurretSyncOffset)
        ;
        lastVisionTimestamp = 0;
        autoSetAction = false;
        autoSetAngle = 0;
        visionLocked = false;
        autoTrack = autoTrackType.autoTrackEnabled;
        disableOdoTrackWithCircle = new Debouncer(Constants.debounceLong);
        disableVisionTrackWithCircle = new Debouncer(Constants.debounceLong * 10);
        triangleDebounce = new Debouncer(1);
        this.imu = imu;
        first = true;
    }

    @Override
    public void HandleUserInput() {
        UserInput = mOp.mOperatorGamepad.right_stick_x;
        // UserInput = Range.scale(UserInput,0,1,Constants.TurretMinAngle, Constants.TurretMaxAngle); // Scale stick to Min Max Turret Angle
        disableOdoTrackWithCircle.In(mOp.mOperatorGamepad.circle);
        if (disableOdoTrackWithCircle.Out()) {
            disableOdoTrackWithCircle.Used();
            if (autoTrack == autoTrackType.autoTrackEnabled) {
                autoTrack = autoTrackType.autoTrackOdoDisabled;
            } else {
                autoTrack = autoTrackType.autoTrackEnabled;
            }
            mOp.mOperatorGamepad.rumbleBlips(1);
        }
        disableVisionTrackWithCircle.In(mOp.mOperatorGamepad.circle);
        if (disableVisionTrackWithCircle.Out()) {
            disableVisionTrackWithCircle.Used();
            if (autoTrack == autoTrackType.autoTrackDisabled) {
                autoTrack = autoTrackType.autoTrackEnabled;
            } else {
                autoTrack = autoTrackType.autoTrackDisabled;
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
        }
        double servoFbPosition = TurretEncoder.getPosition() * Constants.TurretDegreesPerTick; //Degrees ToDo: *Review* May need to apply filter due to spikes seen in data.
        double TurretAnalogPositon = MathUtils.normalizeDegrees(AnalogTurretEncoder.getCurrentPosition(), false)/Constants.TurretGearRatioTurretToServo; //Degrees
        mOp.mTelemetry.addData("TurretServoFb", servoFbPosition);
        mOp.mTelemetry.addData("TurretAnalogPos", TurretAnalogPositon);

        // Order of priorities
        // 1) Auto sets a desired position
        // 2) if a tag is visible, track it
        // 3) turn the turret towards the target using odometry
        // 4) take user input
        boolean applyUpdate = false;
        double NewAngle = 0;
        if (autoSetAction) {
            NewAngle = autoSetAngle;
            applyUpdate = true;
        } else if (VisionTrackingEnabled() && vision != null) {
            mOp.mTelemetry.addData("AprilTag", vision.GetTagClass());
            //mOp.mTelemetry.addData("AprilTag", vision.GetXOffset());
            //mOp.mTelemetry.addData("AprilTag", vision.GetYOffset());
            TurretKinematics.CalcDistanceToTag(vision);
            if (vision.GetTimestamp() > lastVisionTimestamp + VisionRefreshTimeMs) {
                //mOp.mTelemetry.addData("timestamp", vision.GetTimestamp());
                double rotate = vision.GetXOffset();
                //mOp.mTelemetry.addData("rotate", rotate);
                lastVisionTimestamp = vision.GetTimestamp();
                if (Math.abs(rotate) > Constants.TurretDeadbandDegrees) {
                    applyUpdate = true;
//                    if (first) {
//                        firstUpdate = rotate;
//                        first = false;
//                    } else {
//                        if (Math.abs(rotate - firstUpdate) < 1) {
//                            applyUpdate = true;
//                            first = true;
//                        }
//                    }
                    NewAngle = getPosition() + rotate;
                    //mOp.mTelemetry.addData("Turret Pos V", NewPos);
                } else {
                    visionLocked = true;
                }
            }
        } else if (OdometryTrackingEnabled() && currentPose != null) {
            // angle from the robot's current x,y position to the goal
            double angleToGoal = AutoTangent(currentPose.position, goalPosition);
            // get the robot's heading, offset by 180 because the turret is on the back
            double robotHeading = Math.toDegrees(currentPose.heading.toDouble()) - 180;
            // calculate the angle of the turret to point at the goal
            NewAngle = Clamp(MathUtils.normalizeDegrees(robotHeading - angleToGoal, false));
            applyUpdate = true;
        } else if (manualAngleEnable) {
            NewAngle = manualAngle;
            applyUpdate = true;
        } else {
            // scale user input
            double userInputMagnitude = Math.abs(UserInput);
            double userInputSign = Math.signum(UserInput);
            double userInputScalar = userInputSign * Math.pow(userInputMagnitude, userInputExponent);
            double position_change = userInputScalar * mPosChangeRateDegrees;
            if (position_change != 0) {
                // get the last set position and calculate the new position
                NewAngle = servoFbPosition + position_change;
                applyUpdate = true;
            }
        }

        if (applyUpdate) {
            // TODO: *Review* might need fancier logic for this
            visionLocked = Math.abs(NewAngle - servoFbPosition) < Constants.TurretDeadbandDegrees; //Todo *Review* Do we want to indicate locked if we are not checking vision? What if there is an offset in Odometry?
            NewAngle = Clamp(NewAngle);
            SetTurretAngle(NewAngle);
        }

        if (!VisionTrackingEnabled() || System.currentTimeMillis() - lastVisionTimestamp > Constants.TurretVisionLockTimeoutMs) {
            visionLocked = false; //ToDo: *Review* are we only allowing auto launch when vision tracking is enabled?
        }
        //mOp.mTelemetry.addData("AutoPos", autoSetPos);
        mOp.mTelemetry.addData("TurretSetAngle", NewAngle);
        mOp.mTelemetry.addData("VisionLocked", visionLocked);
        mOp.mTelemetry.addData("AutoTrack", autoTrack);
    }

    public boolean Locked() {
        return visionLocked;
    }

    public void ForceUnlock(){
        visionLocked = false;
        // reset the timestamp so we can use the very next update from the camera
        lastVisionTimestamp -= (VisionRefreshTimeMs - 1);
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
        return autoTrack == autoTrackType.autoTrackEnabled;
    }

    protected boolean VisionTrackingEnabled() {
        return autoTrack != autoTrackType.autoTrackDisabled;
//        switch (autoTrack) {
//            case autoTrackDisabled:
//                return false;
//            case autoTrackEnabled:
//            case autoTrackOdoDisabled:
//            default:
//                return true;
//        }
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
                autoTrack = autoTrackType.autoTrackDisabled;
            } else {
                autoTrack = autoTrackType.autoTrackEnabled;
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
