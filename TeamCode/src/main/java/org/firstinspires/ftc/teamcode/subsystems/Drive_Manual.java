package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.Constants;

public class Drive_Manual extends Drive {
    private final com.qualcomm.robotcore.hardware.Gamepad mGamepad;
    private final Debouncer mCircle;
    private final Debouncer mCross;
    private final Limelight3A limelight;
    public Drive_Manual(HydraOpMode op, Imu imu) {
        super(op, imu);
        mGamepad = mOp.mDriverGamepad;
        SetAllMotorPower(0.0);
        SetAllMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mCircle = new Debouncer(Constants.debounce);
        mCross = new Debouncer(Constants.debounce);
        limelight = mOp.mHardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    @Override
    public void Process() {
        double drive;
        double strafe;
        double rotate;
        double rotX;
        double rotY;
        double driveMaxPower;
        double sum;
        double max;
        double frontLeftPower;
        double rearLeftPower;
        double frontRightPower;
        double rearRightPower;
        // get the yaw input from the gyro
        double yaw = 0;
        if (!mImu.Connected()) {
            mOp.mTelemetry.addData("Yaw", "disconnected");
        } else if (mImu.Calibrating()) {
            mOp.mTelemetry.addData("Yaw", "cal");
        } else {
            yaw = mImu.GetYaw();
            mOp.mTelemetry.addData("Yaw", yaw);
        }
        // Get driver controller input
        drive = mGamepad.left_stick_y;
        strafe = -mGamepad.left_stick_x * 1.1;
        double targetx = 0;
        boolean haveTarget = false;
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                haveTarget = true;
                targetx = result.getTx();
            }
        }
        // use the cross button to aid in squaring to the field
        mCross.In(mGamepad.cross);
        if (mCross.Out() && Constants.fieldCentricDrive) {
            rotate = -Math.sin(targetx * Math.PI / 180) * 1.1;
        } else {
            rotate = -mGamepad.right_stick_x;
        }
        rotX = strafe * Math.cos(-yaw / 180 * Math.PI) - drive * Math.sin(-yaw / 180 * Math.PI);
        rotY = strafe * Math.sin(-yaw / 180 * Math.PI) + drive * Math.cos(-yaw / 180 * Math.PI);
        // use the circle button to reset the yaw
        mCircle.In(mGamepad.circle);
        if (mCircle.Out() && Constants.fieldCentricDrive) {
            mCircle.Used();
            mGamepad.rumbleBlips(1);
            mImu.ResetYaw();
        }
        // Set max drive power based on driver input
        if (mGamepad.left_trigger > Constants.trgBtnThresh) {
            // Drive slower for better control
            driveMaxPower = Constants.driveSlow;
        } else if (mGamepad.right_trigger > Constants.trgBtnThresh) {
            // Drive faster for fun
            driveMaxPower = Constants.driveBoosted;
        } else {
            // Normal drive speed
            driveMaxPower = Constants.driveNormal;
        }
        // Scale the output power for the division we are doing later
        sum = Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate);
        if (sum > 1) {
            max = sum;
        } else {
            max = 1;
        }
        // Front left power
        if (Constants.fieldCentricDrive) {
            frontLeftPower = rotY + rotX;
        } else {
            frontLeftPower = drive + strafe;
        }
        frontLeftPower = frontLeftPower + rotate;
        frontLeftPower = frontLeftPower / max;
        // Rear left power
        if (Constants.fieldCentricDrive) {
            rearLeftPower = rotY - rotX;
        } else {
            rearLeftPower = drive - strafe;
        }
        rearLeftPower = rearLeftPower + rotate;
        rearLeftPower = rearLeftPower / max;
        // Front right power
        if (Constants.fieldCentricDrive) {
            frontRightPower = rotY - rotX;
        } else {
            frontRightPower = drive - strafe;
        }
        frontRightPower = frontRightPower - rotate;
        frontRightPower = frontRightPower / max;
        // Rear right power
        if (Constants.fieldCentricDrive) {
            rearRightPower = rotY + rotX;
        } else {
            rearRightPower = drive + strafe;
        }
        rearRightPower = rearRightPower - rotate;
        rearRightPower = rearRightPower / max;
        // Set power to the motors
        mMotDrBkLt.setPower(rearLeftPower * driveMaxPower);
        mMotDrBkRt.setPower(rearRightPower * driveMaxPower);
        mMotDrFrLt.setPower(frontLeftPower * driveMaxPower);
        mMotDrFrRt.setPower(frontRightPower * driveMaxPower);
        mOp.mTelemetry.addData("LeftFront", frontLeftPower);
        mOp.mTelemetry.addData("RightFront", frontRightPower);
        mOp.mTelemetry.addData("LeftRear", rearLeftPower);
        mOp.mTelemetry.addData("RightRear", rearRightPower);
    }
}
