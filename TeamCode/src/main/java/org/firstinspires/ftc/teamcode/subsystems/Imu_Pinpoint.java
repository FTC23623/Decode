package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.AprilTagClass;

public class Imu_Pinpoint extends Imu_Base {
    protected HydraOpMode mOpMode;
    protected final PinpointLocalizer mPinpoint;
    protected final double mAllianceOffsetDeg;
    private boolean isInit;

    public Imu_Pinpoint(HydraOpMode opMode, Pose2d initialPose, AprilTagClass target) {
        super();
        mOpMode = opMode;
        mPinpoint = new PinpointLocalizer(opMode.mHardwareMap, 0, initialPose);
        switch (target) {
            case AprilTagClass_RedGoal:
                mAllianceOffsetDeg = 90;
                break;
            case AprilTagClass_BlueGoal:
                mAllianceOffsetDeg = -90;
                break;
            default:
                mAllianceOffsetDeg = 0;
                break;
        }
        isInit = false;
    }

    @Override
    public boolean Init() {
        if (!isInit) {
            isInit = InitPinpoint(mPinpoint, mOpMode.mTelemetry);
        }
        return isInit;
    }

    public static boolean InitPinpoint(PinpointLocalizer pinpoint, Telemetry telemetry) {
        boolean retval = false;
        final GoBildaPinpointDriver driver = pinpoint.driver;
        GoBildaPinpointDriver.DeviceStatus status = driver.getDeviceStatus();
        switch (status) {
            case READY:
                int version = driver.getDeviceVersion();
                switch (version)
                {
                    case 0:
                    case 1:
                        driver.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.LOCAL_TEST);
                        break;
                    case 2:
                        driver.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
                        break;
                }
                retval = true;
                break;
            case CALIBRATING:
            case NOT_READY:
                telemetry.addLine("Pinpoint calibrating");
                break;
            case FAULT_BAD_READ:
            case FAULT_IMU_RUNAWAY:
            case FAULT_BAD_WRITE_CRC:
            case FAULT_NO_PODS_DETECTED:
            case FAULT_X_POD_NOT_DETECTED:
            case FAULT_Y_POD_NOT_DETECTED:
                telemetry.addLine("Pinpoint critical error");
                break;
        }
        return retval;
    }

    @Override
    public void Process() {
        mPinpoint.update();
    }

    @Override
    public void ResetYaw() {
        Pose2d currentPose = mPinpoint.getPose();
        Pose2d toSet = new Pose2d(currentPose.position, Math.toRadians(mAllianceOffsetDeg));
        mPinpoint.setPose(toSet);
    }

    @Override
    public double GetYaw() {
        // get the absolute robot heading in degrees
        double absHeading = Math.toDegrees(mPinpoint.getPose().heading.toDouble());
        // adjust for alliance
        return absHeading - mAllianceOffsetDeg;
    }
}
