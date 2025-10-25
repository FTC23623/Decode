package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.types.Constants;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.VisionResult;
import org.firstinspires.ftc.teamcode.types.TurretActions;

public class Turret implements Subsystem {
    private final HydraOpMode mOp;
    private final double mPosChangeRate = 0.15;
    private final double mMaxPos = 1;
    private final double mMinPos = 0;
    private double UserInput = 0;
    private Servo TurretServo;
    private final ElapsedTime servoWait;
    private final AnalogInput TurretServoFb;

    public Turret(HydraOpMode opMode) {
        mOp = opMode;
        TurretServo = mOp.mHardwareMap.get(Servo.class,"TurretServo");
        TurretServoFb = mOp.mHardwareMap.get(AnalogInput.class, "TurretServoFb");
        TurretServo.setPosition(0.5);
        servoWait = new ElapsedTime();
        servoWait.reset();
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
        VisionResult vision = mOp.mVision.GetResult();
        if (vision != null) {
            mOp.mTelemetry.addData("AprilTag", vision.GetTagClass());
            mOp.mTelemetry.addData("AprilTag", vision.GetXOffset());
            if (servoWait.milliseconds() > Constants.TurretVisionUpdate) {
                servoWait.reset();
                double CurrentPos = TurretServoFb.getVoltage() / 3.3;
                double rotate = vision.GetXOffset();
                double NewPos = CurrentPos + rotate * Constants.TurretGearRatio / Constants.TurretRange;
                // clamp the new position to the min and max
                NewPos = Clamp(NewPos);
                TurretServo.setPosition(NewPos);
                mOp.mTelemetry.addData("TurretPosition", NewPos);
            }
        } else {
            // scale user input with a constant rate
            double position_change = UserInput * mPosChangeRate;
            // get the last set position and calculate the new position
            double CurrentPos = TurretServo.getPosition();
            double NewPos = CurrentPos + position_change;
            // clamp the new position to the min and max
            NewPos = Clamp(NewPos);
            TurretServo.setPosition(Clamp(NewPos));
            mOp.mTelemetry.addData("Turret Position", NewPos);
        }
    }

    private double Clamp(double position) {
        return Math.min(mMaxPos, Math.max(mMinPos, position));
    }

    /*
     * ROAD RUNNER API
     */
    /**
     * Get a new action object for Road Runner to run
     * @param action: the action to run in this instance
     * @return the action object for RR to use
     */
    public Action GetAction(TurretActions action) {
        return new Turret.RunAction(action);
    }
    /**
     * Runs the supplied action until completion
     */
    public class RunAction implements Action {
        // action this instance will run
        private boolean started = false;
        // run has been called once
        private final TurretActions mAction;

        // construct on the supplied action
        public RunAction(TurretActions action) {
            mAction = action;
        }

        /**
         * Runs the desired action until completion
         * @param packet: ??
         * @return true while the action is running
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            switch (mAction) {
                default:
                    return false;
            }
        }
    }
}
