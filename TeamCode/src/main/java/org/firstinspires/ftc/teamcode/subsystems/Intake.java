package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.IntakeActions;

import java.util.ArrayList;

public class Intake implements Subsystem {
    private final HydraOpMode mOp;
    private final DcMotorEx intakeMotor;
    private final DcMotorEx transferMotor;
    private boolean intakeIn;
    private boolean intakeOut;
    private double intakeOutSpeed;
    private double intakeInSpeed;
    private boolean transferForward;
    private boolean transferReverse;
    private final ArrayList<ArtifactSensor> artifactSensors;
    private boolean sensorRejectEnabled;
    private boolean transferFull;
    private final ElapsedTime rejectionTimer;
    private boolean rejecting = true;

    public Intake(HydraOpMode opmode, boolean enableSensorReject) {
        mOp = opmode;
        intakeMotor = mOp.mHardwareMap.get(DcMotorEx.class, "intakeMotor");
        transferMotor = mOp.mHardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeIn = false;
        intakeOut = false;
        intakeOutSpeed = Constants.intakeMotorMaxOut;
        intakeInSpeed = Constants.intakeMotorMaxIn;
        transferForward = false;
        transferReverse = false;
        this.sensorRejectEnabled = enableSensorReject;
        artifactSensors = new ArrayList<>(3);
        artifactSensors.add(new ArtifactSensor(mOp.mHardwareMap.get(DigitalChannel.class, "artifactSensor1")));
        artifactSensors.add(new ArtifactSensor(mOp.mHardwareMap.get(DigitalChannel.class, "artifactSensor2")));
        artifactSensors.add(new ArtifactSensor(mOp.mHardwareMap.get(DigitalChannel.class, "artifactSensor3")));
        transferFull = false;
        rejectionTimer = new ElapsedTime();
        rejecting = false;
    }

    /**
     * Take user input for next process call
     */
    @Override
    public void HandleUserInput() {
        double right = mOp.mOperatorGamepad.right_trigger;
        double left = mOp.mOperatorGamepad.left_trigger;
        intakeIn = right > Constants.trgBtnThresh;
        intakeInSpeed = Constants.intakeMotorMaxIn * right;
        intakeOut = left > Constants.trgBtnThresh;
        intakeOutSpeed = Constants.intakeMotorMaxOut * left;
        transferForward = mOp.mOperatorGamepad.right_bumper || mOp.mOperatorGamepad.square;
        transferReverse = mOp.mOperatorGamepad.left_bumper;
    }

    /**
     * Start running the intake in
     */
    public void RunIn() {
        intakeIn = true;
        intakeOut = false;
    }

    /**
     * Start running the intake out
     */
    public void RunOut() {
        intakeIn = false;
        intakeOut = true;
    }

    /**
     * Stop running the intake
     */
    public void Stop() {
        intakeIn = false;
        intakeOut = false;
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
        if (sensorRejectEnabled) {
            if (TransferFilled()) {
                // the transfer just filled, we need to reject remaining artifacts
                rejectionTimer.reset();
                rejecting = true;
                if (mOp.mOperatorGamepad != null) {
                    // notify operator transfer has filled
                    mOp.mOperatorGamepad.rumbleBlips(2);
                }
            } else if (rejecting && rejectionTimer.milliseconds() >= Constants.IntakeRejectionTimeMs) {
                // stop rejection now
                rejecting = false;
            }
            if (rejecting) {
                // always want this when rejecting
                intakeIn = false;
                intakeOut = true;
            }
        }
        if (intakeIn) {
            intakeMotor.setPower(intakeInSpeed);
        } else if (intakeOut) {
            intakeMotor.setPower(intakeOutSpeed);
        } else {
            intakeMotor.setPower(0);
        }
        // run the transfer
        if (transferForward) {
            transferMotor.setPower(Constants.TransfertoLaunchPower);
        } else if (intakeIn) {
            transferMotor.setPower(Constants.TransferFromIntakePower);
        } else if (transferReverse) {
            transferMotor.setPower(Constants.TransferToIntakePower);
        } else {
            transferMotor.setPower(0);
        }
    }

    private boolean TransferFilled() {
        boolean full = true;
        // check for artifacts in all positions
        for (ArtifactSensor sensor : artifactSensors) {
            if (!sensor.Full()) {
                // at least one position is still empty, break
                full = false;
                break;
            }
        }
        if (!transferFull && full) {
            // rising edge, capture the state and return true
            transferFull = true;
            return true;
        } else {
            // transfer full state always matches the sensors
            transferFull = full;
            // not a rising edge, so return false
            return false;
        }
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
        private final ElapsedTime timer;

        // construct on the supplied action
        public RunAction(IntakeActions action) {
            mAction = action;
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
                started = true;
                intakeIn = false;
                intakeOut = false;
                transferForward = false;
                transferReverse = false;
                switch (mAction) {
                    case IntakeLoadArtifacts:
                        intakeIn = true;
                        transferForward = true;
                        break;
                    case IntakePushToLauncher:
                        transferForward = true;
                        break;
                    case IntakeReject:
                        intakeOut = true;
                        transferForward = true;
                        timer.reset();
                        break;
                    case IntakeStop:
                    default:
                        break;
                }
            }
            Process();
            if (mAction == IntakeActions.IntakeReject) {
                if (timer.milliseconds() >= 500) {
                    intakeOut = false;
                    Process();
                    return false;
                } else {
                    return true;
                }
            }
            return false;
        }
    }

    private class ArtifactSensor {
        private final DigitalChannel sensor;
        private final ElapsedTime timer;
        private boolean wasFull;

        public ArtifactSensor(DigitalChannel sensor) {
            this.sensor = sensor;
            sensor.setMode(DigitalChannel.Mode.INPUT);
            timer = new ElapsedTime();
            wasFull = false;
        }

        public boolean Full() {
            boolean state = sensor.getState();
            if (state) {
                // beam is broken
                if (!wasFull) {
                    // start timer if this is the rising edge
                    timer.reset();
                    wasFull = true;
                } else {
                    // return true once beam has been broken for desired time
                    return timer.milliseconds() >= Constants.ArtifactDetectionTimeMs;
                }
            } else {
                // reset state
                wasFull = false;
            }
            // beam is not broken or time has not elapsed
            return false;
        }
    }
}
