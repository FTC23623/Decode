package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.types.IntakeActions;
import org.firstinspires.ftc.teamcode.types.LauncherActions;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoNear extends HydrAuto {

    public AutoNear(VisionMode target, boolean flip, int spikeCount, int gate) {
        super(target, flip, spikeCount, gate);
        mBeginPose = FlipPose(-54, 52, 310);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d GPP_WP = FlipPose(34, 35, 90);
        Pose2d GPP = FlipPose(34, 58, 90);
        Pose2d PGP_WP = FlipPose(12, 35, 90);
        Pose2d PGP = FlipPose(12, 58, 90);
        Pose2d PPG_WP = FlipPose(-12, 35, 90);
        Pose2d PPG = FlipPose(-12, 52, 90);
        Pose2d LaunchNear = FlipPose(-25, 24, -40);
        Pose2d End = FlipPose(-14, 36, -40);
        Pose2d GateWP = FlipPose(-6, 50, 180);
        Pose2d Gate = FlipPose(-6, 59, 180);
        Pose2d GateWP2 = FlipPose(4, 50, 0);
        Pose2d Gate2 = FlipPose(4, 59, 0);

        // Action to launch preloaded artifacts
        Action driveToLaunchPreload = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(315))
                .splineToSplineHeading(LaunchNear, FlipTangent(315))
                .build();

        // Action to fetch first spike, return to launch position and launch
        Action fetchPPG;
        if (mGate > 0) {
            fetchPPG = mDrive.actionBuilder(LaunchNear)
                    .setTangent(FlipTangent(0))
                    .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                    .splineToSplineHeading(PPG_WP, FlipTangent(90))
                    .splineToSplineHeading(PPG, FlipTangent(90))
                    .setTangent(FlipTangent(0))
                    .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                    .splineToSplineHeading(GateWP, FlipTangent(-90))
                    .splineToSplineHeading(Gate, FlipTangent(90))
                    .waitSeconds(1.5)
                    .setTangent(FlipTangent(-90))
                    .splineToLinearHeading(LaunchNear, FlipTangent(-180))
                    .build();
        } else {
            fetchPPG = mDrive.actionBuilder(LaunchNear)
                    .setTangent(FlipTangent(0))
                    .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                    .splineToSplineHeading(PPG_WP, FlipTangent(90))
                    .splineToSplineHeading(PPG, FlipTangent(90))
                    .setTangent(FlipTangent(-90))
                    .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                    .splineToLinearHeading(LaunchNear, FlipTangent(-180))
                    .build();
        }

        // Action to fetch second spike, return to launch position and launch
        Action fetchPGP;
        if (mGate > 1) {
            fetchPGP = mDrive.actionBuilder(LaunchNear)
                    .setTangent(FlipTangent(0))
                    .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                    .splineToSplineHeading(PGP_WP, FlipTangent(90))
                    .splineToSplineHeading(PGP, FlipTangent(90))
                    .setTangent(FlipTangent(-150))
                    .splineToSplineHeading(GateWP2, FlipTangent(-90))
                    .splineToSplineHeading(Gate2, FlipTangent(90))
                    .waitSeconds(1.5)
                    .setTangent(FlipTangent(-90))
                    .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                    .splineToLinearHeading(LaunchNear, FlipTangent(-135))
                    .build();
        } else {
            fetchPGP = mDrive.actionBuilder(LaunchNear)
                    .setTangent(FlipTangent(0))
                    .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                    .splineToSplineHeading(PGP_WP, FlipTangent(90))
                    .splineToSplineHeading(PGP, FlipTangent(90))
                    .setTangent(FlipTangent(-90))
                    .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                    .splineToLinearHeading(LaunchNear, FlipTangent(-135))
                    .build();
        }

        // Action to pick up artifacts from third spike and stop
        Action pickupGPP = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(GPP_WP, FlipTangent(90))
                .splineToSplineHeading(GPP, FlipTangent(90))
                .setTangent(FlipTangent(-90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToLinearHeading(LaunchNear, FlipTangent(-135))
                .build();

        // Action to drive from launch position to end position
        Action driveToEnd = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(40))
                .splineToLinearHeading(End, FlipTangent(40))
                .build();

        // Build the auto for launching preloads, fetching from the first spike and launching
        SequentialAction ret = new SequentialAction(
                mTurret.GetDisableAction(true),
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherRunSlow),
                    new SequentialAction(
                            driveToLaunchPreload,
                            mTurret.GetDisableAction(false),
                            mTurret.GetLockAction()
                    )
                ),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mTurret.GetDisableAction(true),
                fetchPPG,
                mTurret.GetDisableAction(false),
                new ParallelAction(
                    mTurret.GetLockAction(),
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher)
                ),
                mLauncher.GetAction(LauncherActions.LauncherLaunch)
        );
        // If more than one spike, add another fetch from the second spike and launch
        if (mSpikeCount > 1) {
            ret = new SequentialAction(
                    ret,
                    mTurret.GetDisableAction(true),
                    fetchPGP,
                    mTurret.GetDisableAction(false),
                    new ParallelAction(
                        mTurret.GetLockAction(),
                        mIntake.GetAction(IntakeActions.IntakePushToLauncher)
                    ),
                    mLauncher.GetAction(LauncherActions.LauncherLaunch)
            );
        }
        // Add pickup of artifacts from final spike
        if (mSpikeCount > 2) {
            ret = new SequentialAction(
                    ret,
                    mTurret.GetDisableAction(true),
                    pickupGPP,
                    mTurret.GetDisableAction(false),
                    new ParallelAction(
                            mTurret.GetLockAction(),
                            mIntake.GetAction(IntakeActions.IntakePushToLauncher)
                    ),
                    mLauncher.GetAction(LauncherActions.LauncherLaunch)
            );
        }
        // when there are 3 spikes we don't launch. If there are less, we need to move off the launch line
        ret = new SequentialAction(
                ret,
                mIntake.GetAction(IntakeActions.IntakeStop),
                driveToEnd
        );
        return ret;
    }
}
