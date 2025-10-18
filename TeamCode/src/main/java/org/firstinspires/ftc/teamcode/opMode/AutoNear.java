package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoNear extends HydrAuto {

    public AutoNear(VisionMode target, boolean flip) {
        super(target, flip);
        mBeginPose = FlipPose(-54, 52, 310);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d GPP_WP = FlipPose(46, 30, 90);
        Pose2d GPP = FlipPose(46, 54, 90);
        Pose2d PGP_WP = FlipPose(22, 30, 90);
        Pose2d PGP = FlipPose(22, 54, 90);
        Pose2d PPG_WP = FlipPose(-4, 30, 90);
        Pose2d PPG = FlipPose(-4, 54, 90);
        Pose2d LaunchNear = FlipPose(-25, 24, -40);
        Pose2d End = FlipPose(10, 20, 0);

        Action driveToLaunch1 = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(315))
                .splineToLinearHeading(LaunchNear, FlipTangent(315))
                .waitSeconds(1.5)
                .splineToLinearHeading(PPG_WP, FlipTangent(90))
                .splineToLinearHeading(PPG, FlipTangent(90))
                .setTangent(FlipTangent(225))
                .splineToLinearHeading(LaunchNear, FlipTangent(-90))
                .waitSeconds(1.5)
                .splineToLinearHeading(PGP_WP, FlipTangent(90))
                .splineToLinearHeading(PGP, FlipTangent(90))
                .setTangent(FlipTangent(225))
                .splineToLinearHeading(LaunchNear, FlipTangent(225))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(25))
                .splineToLinearHeading(GPP_WP, FlipTangent(90))
                .splineToLinearHeading(GPP, FlipTangent(90))
                .setTangent(FlipTangent(225))
                .splineToLinearHeading(LaunchNear, FlipTangent(180))
                .waitSeconds(1.5)
                .splineToLinearHeading(End, FlipTangent(0))
                .build();

        return new SequentialAction(driveToLaunch1);
    }
}
