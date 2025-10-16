package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.IntakeStates;

public class Intake implements Subsystem {
    private final HydraOpMode mOp;
    private final DcMotorEx mMotor;
    private final ElapsedTime mTimeSinceHaveElement;
    //private final ColorRangeSensor mSensor;
    private IntakeStates mState;
    private double mMotorPower;
    private boolean mRunIn;
    private boolean mRunOut;
    private double mRunOutSpeed;
    private double mRunInSpeed;
    private final double mElementDetectionDistance = 0.7;

    public Intake(HydraOpMode opmode) {
        mOp = opmode;
        mMotor = mOp.mHardwareMap.get(DcMotorEx.class, "intakemMotor");
        //mSensor = mOp.mHardwareMap.get(ColorRangeSensor.class, "intakeColorSensor");
        mMotorPower = 0;
        mState = IntakeStates.Idle;
        mTimeSinceHaveElement = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        mRunIn = false;
        mRunOut = false;
        mRunOutSpeed = 0;
        mRunInSpeed = 0;
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
        mState = IntakeStates.Idle;
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
        // use this to exit directly to out regardless of the state
        if (mRunOut && !mRunIn) {
            mState = IntakeStates.Out;
        }
        switch (mState) {
            case Idle:
                // if we want to run in, start the servo and change state
                // otherwise keep the servo off
                if (mRunIn) {
                    mState = IntakeStates.In;
                } else if (mRunOut){
                    mState = IntakeStates.Out;
                } else {
                    mMotorPower = 0;
                    break;
                }
                // fallthrough
            case In:
                // keep the servo running
                mMotorPower = mRunInSpeed;
                if (!mRunIn) {
                    mState = IntakeStates.Idle;
                }
                if (HaveElement()) {
                    // we have detected an element in the intake
                    // transition to the detected state and start our timer
                    mState = IntakeStates.InDetected;
                    mTimeSinceHaveElement.reset();
                }
                break;
            case InDetected:
                // keep running for a bit to make sure the element is fully grasped
                if (mTimeSinceHaveElement.milliseconds() > 500) {
                    // time has elapsed, go to hold
                    mState = IntakeStates.Hold;
                    mRunIn = false;
                }
                break;
            case Hold:
                // keep the servo off until it's time to release it
                if (mRunIn) {
                    mState = IntakeStates.In;
                } else if (mRunOut) {
                    // score it!
                    mState = IntakeStates.Out;
                    mMotorPower = mRunOutSpeed;
                } else if (!HaveElement()) {
                    mState = IntakeStates.Idle;
                } else {
                    mMotorPower = 0;
                }
                break;
            case Out:
                mMotorPower = mRunOutSpeed;
                if (!mRunOut) {
                    mState = IntakeStates.Idle;
                }
                break;
        }
        // setting position on continuous rotation sets the power and direction
        mMotor.setPower(mMotorPower);
        // get the distance from the distance sensor for telemetry
        //double distance = mSensor.getDistance(DistanceUnit.INCH);
        //mOp.mTelemetry.addData("Distance", distance);
        // get the color from the distance sensor for telemetry
        //NormalizedRGBA color = mSensor.getNormalizedColors();
        //mOp.mTelemetry.addData("Red", color.red);
        //mOp.mTelemetry.addData("Green", color.green);
        //mOp.mTelemetry.addData("Blue", color.blue);
    }

    /**
     * Returns whether we have an element in the intake
     * @return true if an element is detected
     */
    public boolean HaveElement() {
        //double distance = mSensor.getDistance(DistanceUnit.INCH);
        //return distance < mElementDetectionDistance;
        return false;
    }
}
