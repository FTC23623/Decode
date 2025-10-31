package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.types.IntakeActions;
import org.firstinspires.ftc.teamcode.types.LauncherActions;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoNearFast extends HydrAuto {

    public AutoNearFast(VisionMode target, boolean flip) {
        super(target, flip);
        mBeginPose = FlipPose(-54, 52, 310);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d GPP_WP = FlipPose(42, 40, 90);
        Pose2d GPP = FlipPose(36, 54, 90);
        Pose2d PGP_WP = FlipPose(19, 36, 90);
        Pose2d PGP = FlipPose(12, 52, 90);
        Pose2d PPG_WP = FlipPose(-4, 32, 90);
        Pose2d PPG = FlipPose(-14, 52, 90);
        Pose2d LaunchNear = FlipPose(-25, 24, -40);
        Pose2d End = FlipPose(-2, 52, 90);

        Action driveToLaunch1 = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(315))
                .splineToSplineHeading(LaunchNear, FlipTangent(315))
                .waitSeconds(.75)
                .build();

        Action driveToLaunch2 = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(PPG_WP, FlipTangent(90))
                .splineToSplineHeading(PPG, FlipTangent(180))
                .splineToSplineHeading(LaunchNear, FlipTangent(-90))
                .waitSeconds(.75)
                .build();

        Action driveToLaunch3 = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(PGP_WP, FlipTangent(90))
                .splineToSplineHeading(PGP, FlipTangent(180))
                .splineToSplineHeading(LaunchNear, FlipTangent(-135))
                .waitSeconds(.75)
                .build();

        Action driveToLaunch4 = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(GPP_WP, FlipTangent(90))
                .splineToSplineHeading(GPP, FlipTangent(180))
                .splineToSplineHeading(LaunchNear, FlipTangent(-135))
                .waitSeconds(.75)
                .build();

        Action driveToEnd = mDrive.actionBuilder(LaunchNear)
                .splineToSplineHeading(End, FlipTangent(90))
                .build();

        return new SequentialAction(
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherRunSlow),
                    driveToLaunch1),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                driveToLaunch2,
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                driveToLaunch3,
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                driveToLaunch4,
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetAction(IntakeActions.IntakeStop),
                driveToEnd);
    }
}
