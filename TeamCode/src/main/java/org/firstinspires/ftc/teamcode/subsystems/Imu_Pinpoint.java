package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.AprilTagClass;

public class Imu_Pinpoint extends Imu_Base {
    protected final PinpointLocalizer mPinpoint;
    protected final double mAllianceOffsetDeg;

    public Imu_Pinpoint(HydraOpMode opMode, Pose2d initialPose, AprilTagClass target) {
        super();
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
    }

    @Override
    public void Process() {
        mPinpoint.update();
    }

    public void ResetYaw() {
        Pose2d currentPose = mPinpoint.getPose();
        Pose2d toSet = new Pose2d(currentPose.position, Math.toRadians(mAllianceOffsetDeg));
        mPinpoint.setPose(toSet);
    }

    public double GetYaw() {
        // get the absolute robot heading in degrees
        double absHeading = Math.toDegrees(mPinpoint.getPose().heading.toDouble());
        // adjust for alliance
        return absHeading - mAllianceOffsetDeg;
    }
}
