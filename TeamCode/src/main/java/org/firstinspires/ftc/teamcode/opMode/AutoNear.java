package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.types.IntakeActions;
import org.firstinspires.ftc.teamcode.types.LauncherActions;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoNear extends HydrAuto {

    public AutoNear(VisionMode target, boolean flip, int spikeCount) {
        super(target, flip, spikeCount);
        mBeginPose = FlipPose(-54, 52, 310);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d GPP_WP = FlipPose(35, 40, 90);
        Pose2d GPP = FlipPose(35, 54, 90);
        Pose2d PGP_WP = FlipPose(12, 36, 90);
        Pose2d PGP = FlipPose(12, 51, 90);
        Pose2d PPG_WP = FlipPose(-12, 32, 90);
        Pose2d PPG = FlipPose(-20, 51, 90);
        Pose2d LaunchNear = FlipPose(-25, 24, -40);
        Pose2d End = FlipPose(-2, 52, 90);

        // Action to launch preloaded artifacts
        Action driveToLaunchPreload = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(315))
                .splineToSplineHeading(LaunchNear, FlipTangent(315))
                .waitSeconds(.75)
                .build();

        // Action to fetch first spike, return to launch position and launch
        Action fetchPPG = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(PPG_WP, FlipTangent(90))
                .splineToSplineHeading(PPG, FlipTangent(180))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToSplineHeading(LaunchNear, FlipTangent(-90))
                .waitSeconds(.75)
                .build();

        // Action to fetch second spike, return to launch position and launch
        Action fetchPGP = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(PGP_WP, FlipTangent(90))
                .splineToSplineHeading(PGP, FlipTangent(90))
                .setTangent(FlipTangent(-90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToSplineHeading(LaunchNear, FlipTangent(-135))
                .waitSeconds(.75)
                .build();

        // Action to pick up artifacts from third spike and stop
        Action pickupGPP = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(GPP_WP, FlipTangent(90))
                .splineToSplineHeading(GPP, FlipTangent(90))
                // wait here and then reject / stop intake
                .waitSeconds(1)
                .build();

        // Action to drive from launch position to end position
        Action driveToEnd = mDrive.actionBuilder(LaunchNear)
                .splineToSplineHeading(End, FlipTangent(90))
                .build();

        // Build the auto for launching preloads, fetching from the first spike and launching
        SequentialAction ret = new SequentialAction(
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherRunSlow),
                    driveToLaunchPreload),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                fetchPPG,
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch)
        );
        // If more than one spike, add another fetch from the second spike and launch
        if (mSpikeCount > 1) {
            ret = new SequentialAction(
                    ret,
                    fetchPGP,
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherLaunch)
            );
        }
        // Add pickup of artifacts from final spike
        if (mSpikeCount > 2) {
            ret = new SequentialAction(
                    ret,
                    pickupGPP,
                    mIntake.GetAction(IntakeActions.IntakeReject)
            );
        }
        // when there are 3 spikes we don't launch. If there are less, we need to move off the launch line
        if (mSpikeCount < 3) {
            ret = new SequentialAction(
                    ret,
                    mIntake.GetAction(IntakeActions.IntakeStop),
                    driveToEnd
            );
        }
        return ret;
    }
}
