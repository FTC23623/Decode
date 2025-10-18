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
        Pose2d GPP_WP = FlipPose(40, 30, 90);
        Pose2d GPP = FlipPose(40, 54, 90);
        Pose2d PGP_WP = FlipPose(16, 30, 90);
        Pose2d PGP = FlipPose(16, 54, 90);
        Pose2d PPG_WP = FlipPose(-7, 30,90);
        Pose2d PPG = FlipPose(-7, 54, 90);
        Pose2d End = FlipPose(30,10,0);

        Action driveToLaunch1 = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(180))
                .splineToLinearHeading(Launch, FlipTangent(180))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180))
                .splineToLinearHeading(GPP_WP, FlipTangent(135))
                .setTangent(FlipTangent(90))
                .splineToLinearHeading(GPP, FlipTangent(90))
                .setTangent(FlipTangent(-90))
                .splineToLinearHeading(Launch, FlipTangent(0))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180))
                .splineToLinearHeading(PGP_WP, FlipTangent(135))
                .setTangent(FlipTangent(90))
                .splineToLinearHeading(PGP, FlipTangent(90))
                .setTangent(FlipTangent(-90))
                .splineToLinearHeading(Launch, FlipTangent(0))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180))
                .splineToLinearHeading(PPG_WP, FlipTangent(135))
                .setTangent(FlipTangent(90))
                .splineToLinearHeading(PPG, FlipTangent(90))
                .setTangent(FlipTangent(-90))
                .splineToLinearHeading(Launch, FlipTangent(0))
                .waitSeconds(1.5)
                .splineToLinearHeading(End, FlipTangent(180))
                .build();

        return new SequentialAction(driveToLaunch1);
    }
}
