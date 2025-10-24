package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.IntakeActions;

public class Intake implements Subsystem {
    private final HydraOpMode mOp;
    private final DcMotorEx mMotor;
    private final DcMotorEx mTransferMotor;
    private boolean mRunIn;
    private boolean mRunOut;
    private double mRunOutSpeed;
    private double mRunInSpeed;
    private boolean launching = false;

    public Intake(HydraOpMode opmode) {
        mOp = opmode;
        mMotor = mOp.mHardwareMap.get(DcMotorEx.class, "intakeMotor");
        mTransferMotor = mOp.mHardwareMap.get(DcMotorEx.class, "transferMotor");
        mTransferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        mRunIn = false;
        mRunOut = false;
        mRunOutSpeed = Constants.intakeMotorMaxOut;
        mRunInSpeed = Constants.intakeMotorMaxIn;
    }

    /**
     * Take user input for next process call
     */
    @Override
    public void HandleUserInput() {
        double right = mOp.mOperatorGamepad.right_trigger;
        double left = mOp.mOperatorGamepad.left_trigger;
        mRunIn = right > Constants.trgBtnThresh;
        mRunInSpeed = Constants.intakeMotorMaxIn * right;
        mRunOut = left > Constants.trgBtnThresh;
        mRunOutSpeed = Constants.intakeMotorMaxOut * left;
        launching = mOp.mOperatorGamepad.right_bumper;
    }

    /**
     * Start running the intake in
     */
    public void RunIn() {
        mRunIn = true;
        mRunOut = false;
    }

    /**
     * Start running the intake out
     */
    public void RunOut() {
        mRunIn = false;
        mRunOut = true;
    }

    /**
     * Stop running the intake
     */
    public void Stop() {
        mRunIn = false;
        mRunOut = false;
    }

    @Override
    public boolean Init() {
        return true;
    }

    /**
     * Process for running the intake
     */
    @Override
    public void Process() {
        // run the intake
        if (mRunIn) {
            mMotor.setPower(mRunInSpeed);
        } else if (mRunOut) {
            mMotor.setPower(mRunOutSpeed);
        } else {
            mMotor.setPower(0);
        }
        // run the transfer
        if(launching){
            mTransferMotor.setPower(Constants.TransfertoLaunchPower);
        }
        else if(mRunIn){
            mTransferMotor.setPower(Constants.TransferFromIntakePower);
        }
       else if(mRunOut) {
            mTransferMotor.setPower(Constants.TransferToIntakePower);
        }
       else{
            mTransferMotor.setPower(0);
        }
        mOp.mTelemetry.addData("Intake Current", mMotor.getCurrent(CurrentUnit.MILLIAMPS));
        mOp.mTelemetry.addData("Transfer Current", mTransferMotor.getCurrent(CurrentUnit.MILLIAMPS));
    }

    /*
     * ROAD RUNNER API
     */
    /**
     * Get a new action object for Road Runner to run
     * @param action: the action to run in this instance
     * @return the action object for RR to use
     */
    public Action GetAction(IntakeActions action) {
        return new RunAction(action);
    }
    /**
     * Runs the supplied action until completion
     */
    public class RunAction implements Action {
        // action this instance will run
        private boolean started = false;
        // run has been called once
        private final IntakeActions mAction;

        // construct on the supplied action
        public RunAction(IntakeActions action) {
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
