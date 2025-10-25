package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoFar extends HydrAuto {
    public AutoFar(VisionMode target, boolean flip) {
        super(target, flip);
        mBeginPose = FlipPose(60.0, 15.0, 0.0);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d Launch = FlipPose(55, 15, -20);
        Pose2d GPP_WP = FlipPose(34, 30, 90);
        Pose2d GPP = FlipPose(40, 54, 90);
        Pose2d PGP_WP = FlipPose(12, 30, 90);
        Pose2d PGP = FlipPose(16, 54, 90);
        Pose2d PPG_WP = FlipPose(-11, 33,90);
        Pose2d PPG = FlipPose(-5, 54, 90);
        Pose2d End = FlipPose(30,15,0);

        Action driveToLaunch1 = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(Launch, FlipTangent(180))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(GPP_WP, FlipTangent(90))
                .setTangent(FlipTangent(90))
                .splineToSplineHeading(GPP, FlipTangent(0))
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(Launch, FlipTangent(-90))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(PGP_WP, FlipTangent(90))
                .setTangent(FlipTangent(90))
                .splineToSplineHeading(PGP, FlipTangent(0))
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(Launch, FlipTangent(-90))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(PPG_WP, FlipTangent(90))
                .setTangent(FlipTangent(90))
                .splineToSplineHeading(PPG, FlipTangent(0))
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(Launch, FlipTangent(-90))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(End, FlipTangent(180))
                .build();

        return new SequentialAction(driveToLaunch1);
    }
}
