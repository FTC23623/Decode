package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.types.IntakeActions;
import org.firstinspires.ftc.teamcode.types.LauncherActions;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoLoadingZone extends HydrAuto {
    public AutoLoadingZone(VisionMode target, boolean flip, int spikeCount) {
        super(target, flip, spikeCount, 0);
        mBeginPose = FlipPose(60.0, 15.0, 0.0);
    }

    @Override
    protected SequentialAction CreateAuto() {
        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d Launch = FlipPose(55, 15, -20);

        Action launchPreload = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(Launch, FlipTangent(180))
                .build();

        // Build the auto for launching preloads, fetching artifacts from the first spike and launching
        SequentialAction ret =  new SequentialAction(
                mTurret.GetDisableAction(true),
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherRunFast),
                    new SequentialAction(
                            launchPreload,
                            mTurret.GetDisableAction(false),
                            mTurret.GetLockAction()
                    )
                ),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                LoadingZoneSequence(Launch, true)
        );
        // If more than one spike, add another fetch from the second spike and launch
        if (mSpikeCount > 1) {
            ret = new SequentialAction(
                    ret,
                    LoadingZoneSequence(Launch, true)
            );
        }
        // Add pickup of artifacts from final spike
        if (mSpikeCount > 2) {
            ret = new SequentialAction(
                    ret,
                    LoadingZoneSequence(Launch, true)
            );
        }
        ret = new SequentialAction(
                ret,
                LoadingZoneSequence(Launch, false)
        );
        return ret;
    }

    private SequentialAction LoadingZoneSequence(Pose2d LaunchPos, boolean driveToLaunch) {
        Pose2d LoadingZone = FlipPose(59,55,90);
        Pose2d LoadingZone_WP= FlipPose(59, 40, 90);

        // fetch and drive to waypoint
        Action fetch = mDrive.actionBuilder(LaunchPos)
                .setTangent(FlipTangent(90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(90))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90))
                .afterTime(.75, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(-90))
                .build();

        // fetch and launch
        Action goToLaunch =  mDrive.actionBuilder(LaunchPos)
                .setTangent(FlipTangent(90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(90))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90))
                .afterTime(.75, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(-90))
                .splineToSplineHeading(LaunchPos, FlipTangent(-90))
                .build();

        // if we're launching, continue to launch point, re-enable turret and launch
        // otherwise stop and re-enable turret
        if (driveToLaunch) {
            return new SequentialAction(
                    mTurret.GetDisableAction(true),
                    goToLaunch,
                    mTurret.GetDisableAction(false),
                    new ParallelAction(
                            mTurret.GetLockAction(),
                            mIntake.GetAction(IntakeActions.IntakePushToLauncher)
                    ),
                    mLauncher.GetAction(LauncherActions.LauncherLaunch)
            );
        } else {
            return new SequentialAction(
                    mTurret.GetDisableAction(true),
                    fetch,
                    mTurret.GetDisableAction(false)
            );
        }
    }
}
