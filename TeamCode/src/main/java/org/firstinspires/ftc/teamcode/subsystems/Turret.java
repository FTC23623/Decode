package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.types.Constants;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.VisionResult;

public class Turret implements Subsystem {
    private final HydraOpMode mOp;
    private final double mPosChangeRate = 0.15;
    private final double mMaxPos = 1;
    private final double mMinPos = 0;
    private double UserInput = 0;
    private Servo TurretServo;
    private final AnalogInput TurretServoFb;
    private long lastVisionTimestamp;
    private boolean autoSetAction;
    private double autoSetPos;
    private boolean visionLocked;

    public Turret(HydraOpMode opMode) {
        mOp = opMode;
        TurretServo = mOp.mHardwareMap.get(Servo.class,"TurretServo");
        TurretServoFb = mOp.mHardwareMap.get(AnalogInput.class, "TurretServoFb");
        TurretServo.setPosition(0.5);
        lastVisionTimestamp = 0;
        autoSetAction = false;
        autoSetPos = 0;
        visionLocked = false;
    }

    @Override
    public boolean Init() {
        return true;
    }

    @Override
    public void HandleUserInput() {
        UserInput = mOp.mOperatorGamepad.right_stick_x;
    }

    @Override
    public void Process() {
        VisionResult vision = null;
        if (mOp.mVision != null) {
            vision = mOp.mVision.GetResult();
        }
        double servoFbPosition = GetPositionFromFb();
        mOp.mTelemetry.addData("TurretServoFb", servoFbPosition);
        if (autoSetAction) {
            TurretServo.setPosition(autoSetPos);
        }
        else if (vision != null) {
            mOp.mTelemetry.addData("AprilTag", vision.GetTagClass());
            mOp.mTelemetry.addData("AprilTag", vision.GetXOffset());
            mOp.mTelemetry.addData("AprilTag", vision.GetYOffset());
            CalcDistanceToTag(vision);
            if (vision.GetTimestamp() > lastVisionTimestamp + 250) {
                mOp.mTelemetry.addData("timestamp", vision.GetTimestamp());
                double rotate = vision.GetXOffset();
                mOp.mTelemetry.addData("rotate", rotate);
                lastVisionTimestamp = vision.GetTimestamp();
                if (Math.abs(rotate) > 1) {
                    double NewPos = servoFbPosition + CalcPositionOffsetAngle(rotate);
                    // clamp the new position to the min and max
                    NewPos = Clamp(NewPos);
                    TurretServo.setPosition(NewPos);
                    visionLocked = false;
                    mOp.mTelemetry.addData("Turret Pos V", NewPos);
                } else {
                    visionLocked = true;
                }
            }
        } else {
            // scale user input with a constant rate
            double position_change = UserInput * mPosChangeRate;
            if (position_change != 0) {
                // get the last set position and calculate the new position
                double NewPos = servoFbPosition + position_change;
                // clamp the new position to the min and max
                NewPos = Clamp(NewPos);
                TurretServo.setPosition(Clamp(NewPos));
                mOp.mTelemetry.addData("Turret Pos U", NewPos);
            }
        }
        if (System.currentTimeMillis() - lastVisionTimestamp > Constants.TurretVisionLockTimeoutMs) {
            visionLocked = false;
        }
        mOp.mTelemetry.addData("AutoPos", autoSetPos);
        mOp.mTelemetry.addData("AutoAction", autoSetAction);
        mOp.mTelemetry.addData("VisionLocked", visionLocked);
    }

    public boolean Locked() {
        return visionLocked;
    }

    private double Clamp(double position) {
        return Math.min(mMaxPos, Math.max(mMinPos, position));
    }

    private double GetPositionFromFb() {
        return Clamp(1 - TurretServoFb.getVoltage() / Constants.TurretServoAnalogRangeVolts);
    }

    private double CalcDistanceToTag(VisionResult vision) {
        double targetOffsetAngle_Vertical = vision.GetYOffset();
        double angleToGoalDegrees = Constants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        //calculate distance
        double distanceFromLimelightToGoalInches = (Constants.goalHeightInches - Constants.limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        mOp.mTelemetry.addData("GoalDistance", distanceFromLimelightToGoalInches);
        return distanceFromLimelightToGoalInches;
    }

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
    /**
     * Runs the supplied action until completion
     */
    public class RunLockAction implements Action {
        // run has been called once
        protected boolean started = false;
        public RunLockAction() {
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !visionLocked;
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
}
