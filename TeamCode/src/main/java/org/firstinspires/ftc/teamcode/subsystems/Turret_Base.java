package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class Turret_Base implements Subsystem {
    protected final HydraOpMode mOp;
    protected int mAllianceFactor;
    protected final VoltageSensor voltageSensor;
    protected final AbsoluteAnalogEncoder AnalogTurretEncoder;
    protected Motor.Encoder TurretEncoder;
    protected final double mPosChangeRate = 0.2;
    protected double TurretSyncOffset = 0.0;
    protected long lastVisionTimestamp;
    protected boolean autoSetAction;
    protected double autoSetPos;
    protected boolean visionLocked;
    protected boolean disableAutoTrack;
    protected final Debouncer circleDebounce;
    protected final Debouncer triangleDebounce;
    protected final Imu imu;
    protected double UserInput = 0;
    public static int VisionRefreshTimeMs = 10;


    public Turret_Base(HydraOpMode opMode, Imu imu, VisionMode target) {
        mOp = opMode;
        voltageSensor = mOp.mHardwareMap.get(VoltageSensor.class, "Control Hub");
        // Adjust target location for alliance
        if (target == VisionMode.VisionMode_BlueGoal) {
            mAllianceFactor = 1;
        }
        else {
            mAllianceFactor = -1;
        }
        AnalogTurretEncoder = new AbsoluteAnalogEncoder(mOp.mHardwareMap,"TurretServoFb", Constants.TurretServoAnalogRangeVolts, AngleUnit.DEGREES)
                .zero(Constants.TurretEncoderOffset)
                .setReversed(true)
        ;
        TurretEncoder = new Motor(mOp.mHardwareMap, "leftBack").encoder
                .setDirection(Motor.Direction.REVERSE) // ToDo: Set based on Encoder orientation and Positive rotation convention
                .overrideResetPos((int) TurretSyncOffset)
        ;
        lastVisionTimestamp = 0;
        autoSetAction = false;
        autoSetPos = 0;
        visionLocked = false;
        disableAutoTrack = false;
        circleDebounce = new Debouncer(Constants.debounceLong);
        triangleDebounce = new Debouncer(1);
        this.imu = imu;
    }

    @Override
    public void HandleUserInput() {
        //UserInput = mOp.mOperatorGamepad.right_stick_x;
        //UserInput = Range.scale(UserInput,0,1,Constants.TurretMinAngle, Constants.TurretMaxAngle); // Scale stick to Min Max Turret Angle
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

    public abstract void GoHome();

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

    public abstract Action GetSetAction(double position);

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
            disableAutoTrack = disable;
            return false;
        }
    }
}
