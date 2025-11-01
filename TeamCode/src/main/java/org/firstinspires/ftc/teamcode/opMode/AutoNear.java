package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.types.IntakeActions;
import org.firstinspires.ftc.teamcode.types.LauncherActions;
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
        Pose2d GPP_WP = FlipPose(35, 40, 90);
        Pose2d GPP = FlipPose(35, 54, 90);
        Pose2d PGP_WP = FlipPose(12, 36, 90);
        Pose2d PGP = FlipPose(6, 51, 90);
        Pose2d PPG_WP = FlipPose(-11, 32, 90);
        Pose2d PPG = FlipPose(-18, 48, 90);
        Pose2d LaunchNear = FlipPose(-25, 24, -40);
        Pose2d End = FlipPose(-2, 52, 90);

        Action driveToLaunchPreload = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(315))
                .splineToSplineHeading(LaunchNear, FlipTangent(315))
                .waitSeconds(.75)
                .build();

        Action fetchPPG = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(PPG_WP, FlipTangent(90))
                .splineToSplineHeading(PPG, FlipTangent(180))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .afterTime(1.5, mIntake.GetAction(IntakeActions.IntakeStop))
                .splineToSplineHeading(LaunchNear, FlipTangent(-90))
                .waitSeconds(.75)
                .build();

        Action fetchPGP = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(PGP_WP, FlipTangent(90))
                .splineToSplineHeading(PGP, FlipTangent(180))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .afterTime(1.5, mIntake.GetAction(IntakeActions.IntakeStop))
                .splineToSplineHeading(LaunchNear, FlipTangent(-135))
                .waitSeconds(.75)
                .build();

        Action pickupGPP = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(GPP_WP, FlipTangent(90))
                .splineToSplineHeading(GPP, FlipTangent(90))
                //.splineToSplineHeading(LaunchNear, FlipTangent(-135))
                //.waitSeconds(.75)
                .build();

        Action driveToEnd = mDrive.actionBuilder(LaunchNear)
                .splineToSplineHeading(End, FlipTangent(90))
                .build();

        return new SequentialAction(
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherRunSlow),
                    driveToLaunchPreload),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                fetchPPG,
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                fetchPGP,
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                pickupGPP/*,
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetAction(IntakeActions.IntakeStop),
                driveToEnd*/
                );
    }
}
