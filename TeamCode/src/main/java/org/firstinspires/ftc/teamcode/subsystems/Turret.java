package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.types.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.VisionResult;

import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Config
public class Turret implements Subsystem {
    private final HydraOpMode mOp;
    private final double mPosChangeRate = 0.2;
    public static double mMaxPos = 0.83802817; //0.5 + 120/177.5 * 0.5 (120deg)
    public static double mMinPos = 0.33802817; // 120/177.5 * 0.5 (-120deg)
    private double UserInput = 0;
    private final Servo TurretServo;
    //private final AnalogInput TurretServoFb;
    public AbsoluteAnalogEncoder AnalogTurretEncoder;
    public Motor.Encoder TurretEncoder;
    public static double TurretSyncOffset = 0.0;
    public static boolean TurretSynced = false; // Indicates turret encoder is synced to servo absolute encoder.
    private long lastVisionTimestamp;
    private boolean autoSetAction;
    private double autoSetPos;
    private boolean visionLocked;
    private boolean disableAutoTrack;
    public static int VisionRefreshTimeMs = 10;
    private final Debouncer circleDebounce;
    private final Debouncer triangleDebounce;
    //private final Debouncer squareDebounce;
    private boolean first;
    private double firstUpdate;
    private final Imu imu;
    private boolean initEncoder = true;
    protected static double mAllianceFactor = -1;



    public Turret(HydraOpMode opMode, Imu imu, VisionMode target) {
        mOp = opMode;
        TurretServo = mOp.mHardwareMap.get(Servo.class,"TurretServo");
        //TurretServoFb = mOp.mHardwareMap.get(AnalogInput.class, "TurretServoFb");

        AnalogTurretEncoder = new AbsoluteAnalogEncoder(mOp.mHardwareMap,"TurretServoFb",Constants.TurretServoAnalogRangeVolts, AngleUnit.DEGREES)
                .zero(Constants.TurretEncoderOffset)
                .setReversed(true) // ToDo: Set based on observation.
        ;

        TurretEncoder = new Motor(mOp.mHardwareMap, "leftBack").encoder
                .setDirection(Motor.Direction.REVERSE) // ToDo: Set based on Encoder orientation and Positive rotation convention
                .overrideResetPos((int) TurretSyncOffset)
        ;
        // Adjust target location for alliance
        if (target == VisionMode.VisionMode_BlueGoal) {
            mAllianceFactor = 1;
        }
        else {
            mAllianceFactor = -1;
        }
        lastVisionTimestamp = 0;
        autoSetAction = false;
        autoSetPos = 0;
        visionLocked = false;
        disableAutoTrack = false;
        circleDebounce = new Debouncer(Constants.debounceLong);
        triangleDebounce = new Debouncer(1);
        //squareDebounce = new Debouncer(2);
        first = true;
        this.imu = imu;
    }

    @Override
    public boolean Init() {  //ToDo: ?????? Does this really get called during Opmode Init? ????????
        //TurretEncoder.overrideResetPos(0); // Assumes Turret is at Zero on Init
        return true;
    }
/*
    // Synchronize throughbore encoder with Axon Servo feedback
    public void resetTurretEncoder() {
        if (!TurretSynced) {
            if (AnalogTurretEncoder.getVoltage() > 0.001) {
                TurretEncoder.overrideResetPos(0);
                TurretSyncOffset = TurretEncoder.getPosition() - (MathUtils.normalizeDegrees(AnalogTurretEncoder.getCurrentPosition(), false) * Constants.TurretDegreesPerTick); //ToDo: The Math should have a gear ratio from Servo to encoder in it.
                TurretEncoder.overrideResetPos((int) TurretSyncOffset);
                TurretSynced = true;
            }
        }
    }
*/

    @Override
    public void HandleUserInput() {
        UserInput = mOp.mOperatorGamepad.right_stick_x;
        circleDebounce.In(mOp.mOperatorGamepad.circle);
        if (circleDebounce.Out()) {
            circleDebounce.Used();
            disableAutoTrack = !disableAutoTrack;
            mOp.mOperatorGamepad.rumbleBlips(1);
        }
        triangleDebounce.In(mOp.mOperatorGamepad.triangle);
        if (triangleDebounce.Out()) {
            triangleDebounce.Used();
            GoHome();
        }
        //squareDebounce.In(mOp.mDriverGamepad.square);
        //if (squareDebounce.Out()) {
        //    GoHome();
        //}
    }

    @Override
    public void Process() {
        // Initialize turret encoder on first loop
        if (initEncoder){ //Todo: Verify that this will work. Does turret get to home before encoder is reset? Better would be to run this at Init.
            TurretServo.setPosition(0.5); // Send Turret Home
            TurretEncoder.overrideResetPos(0); // Assumes Turret is at Zero on Init;
            initEncoder = false;
        }

        VisionResult vision = null;
        if (mOp.mVision != null) {
            vision = mOp.mVision.GetResult();
        }
        Pose2d currentPose = null;
        if (imu != null) {
            currentPose = imu.GetPose();
        }
        //double servoFbPosition = GetPositionFromFb();
        double servoFbPosition = TurretEncoder.getPosition() * Constants.TurretDegreesPerTick; //Degrees
        double TurretAnalogPositon = MathUtils.normalizeDegrees(AnalogTurretEncoder.getCurrentPosition(), false)/Constants.TurretGearRatioTurretToServo; //Degrees
        mOp.mTelemetry.addData("TurretServoFb", servoFbPosition);
        mOp.mTelemetry.addData("TurretAnalogPos", TurretAnalogPositon);

        // Order of priorities
        // 1) Auto sets a desired position
        // 2) if a tag is visible, track it
        // 3) turn the turret towards the target using odometry
        // 4) take user input
        if (autoSetAction) {
            TurretServo.setPosition(autoSetPos);
        } else if (!disableAutoTrack && vision != null) {
            mOp.mTelemetry.addData("AprilTag", vision.GetTagClass());
            //mOp.mTelemetry.addData("AprilTag", vision.GetXOffset());
            //mOp.mTelemetry.addData("AprilTag", vision.GetYOffset());
            TurretKinematics.CalcDistanceToTag(vision);
            if (vision.GetTimestamp() > lastVisionTimestamp + VisionRefreshTimeMs) {
                //mOp.mTelemetry.addData("timestamp", vision.GetTimestamp());
                double rotate = vision.GetXOffset();
                //mOp.mTelemetry.addData("rotate", rotate);
                lastVisionTimestamp = vision.GetTimestamp();
                if (Math.abs(rotate) > 1) {
                    double update = CalcPositionOffsetAngle(rotate);
                    boolean applyUpdate = false;
                    if (first) {
                        firstUpdate = update;
                        first = false;
                    } else {
                        if (Math.abs(update - firstUpdate) < 1) {
                            applyUpdate = true;
                            first = true;
                        }
                    }
                    double NewPos = TurretServo.getPosition() + update; //servoFbPosition + update;
                    //double NewPos = servoFbPosition + update; // in Degrees
                    // clamp the new position to the min and max
                    NewPos = Clamp(NewPos);
                    if (applyUpdate) {
                        TurretServo.setPosition(NewPos); // ToDo: Seperate Set Position from Nested IF and use common variable to set in each IF case.
                        visionLocked = false;
                    }
                    //mOp.mTelemetry.addData("Turret Pos V", NewPos);
                } else {
                    visionLocked = true;
                }
            }
        } else if (!disableAutoTrack && currentPose != null){
            // TODO: GET GOAL POSITIONS AND SELECT BASED ON ALLIANCE
            Vector2d goalPosition = new Vector2d(-72, 72 * mAllianceFactor);
            // angle from the robot's current x,y position to the goal
            double angleToGoal = AutoTangent(currentPose.position, goalPosition);
            // get the robot's heading, offset by 180 because the turret is on the back
            double robotHeading = Math.toDegrees(currentPose.heading.toDouble()) - 180;
            // calculate the angle of the turret to point at the goal
            double turretAngleToSet = MathUtils.normalizeDegrees(robotHeading - angleToGoal, true); // ToDo: Seperate Set Position from Nested IF and use common variable to set in each IF case.
        } else {
            // scale user input with a constant rate
            double position_change = UserInput * mPosChangeRate; //Todo: Change to angle based rather than servo command based.
            if (position_change != 0) {
                // get the last set position and calculate the new position
                double NewPos = TurretServo.getPosition() + position_change;
                //double NewPos = servoFbPosition + position_change; // in Degrees
                // clamp the new position to the min and max
                NewPos = Clamp(NewPos);
                TurretServo.setPosition(Clamp(NewPos)); // ToDo: Seperate Set Position from Nested IF and use common variable to set in each IF case.
                //mOp.mTelemetry.addData("Turret Pos U", NewPos);
            }
        }
        if (disableAutoTrack || System.currentTimeMillis() - lastVisionTimestamp > Constants.TurretVisionLockTimeoutMs) {
            visionLocked = false;
        }
        //mOp.mTelemetry.addData("AutoPos", autoSetPos);
        //mOp.mTelemetry.addData("AutoAction", autoSetAction);
        mOp.mTelemetry.addData("VisionLocked", visionLocked);
        mOp.mTelemetry.addData("AutoTrack", !disableAutoTrack);
    }

    private static double AutoTangent(Vector2d start, Vector2d end) {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public boolean Locked() {
        return visionLocked;
    }

    public void ForceUnlock(){
        visionLocked = false;
        // reset the timestamp so we can use the very next update from the camera
        lastVisionTimestamp -= (VisionRefreshTimeMs - 1);
    }

    public void GoHome() {
        autoSetAction = true;
        autoSetPos = 0.5;
        Process();
        autoSetAction = false;
    }

    private double Clamp(double position) {
        return Math.min(mMaxPos, Math.max(mMinPos, position));
    }

   // private double GetPositionFromFb() {
   //     return Clamp(1 - TurretServoFb.getVoltage() / Constants.TurretServoAnalogRangeVolts);
   // }

    private double CalcPositionOffsetAngle(double degrees) {
        return degrees * Constants.TurretGearRatio / Constants.TurretRange;
    }

    private double CalcAbsolutePositionFromAngle(double degrees) {
        // convert the angle to 0 to max since servo is 0 to 1
        degrees = degrees * Constants.TurretGearRatio + Constants.TurretRange / 2;
        // clamp the value to the range
        degrees = Math.max(0, Math.min(Constants.TurretRange, degrees));
        return degrees / Constants.TurretRange;
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
        return new Turret.RunLockAction();
    }

    public Action GetSetAction(double position) {
        return new Turret.RunSetAction(position);
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
                autoSetPos = CalcAbsolutePositionFromAngle(angleToSet);
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

    public class RunDisableAction implements Action {
        boolean disable;

        public RunDisableAction(boolean disable) {
            this.disable = disable;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            disableAutoTrack = disable;
            return false;
        }
    }

}
