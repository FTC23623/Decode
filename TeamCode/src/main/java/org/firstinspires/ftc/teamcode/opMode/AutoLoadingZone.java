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
        super(target, flip, spikeCount, false);
        mBeginPose = FlipPose(60.0, 15.0, 0.0);
    }

    @Override
    protected SequentialAction CreateAuto() {
        boolean pickupAtEnd = true;
        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d Launch = FlipPose(55, 15, -20);
        Pose2d GPP_WP = FlipPose(34, 30, 90);
        Pose2d GPP = FlipPose(34, 58, 90);
        Pose2d PGP_WP = FlipPose(12, 30, 90);
        Pose2d PGP = FlipPose(12, 58, 90);
        Pose2d PPG_WP = FlipPose(-12, 35, 90);
        Pose2d PPG = FlipPose(-12, 48, 90);
        Pose2d End = FlipPose(30, 15, 0);
        Pose2d LoadingZone = FlipPose(59,55,90);
        Pose2d LoadingZone_WP= FlipPose(59, 40, 90);

        Action launchPreload = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(Launch, FlipTangent(180))
                .build();

        Action fetchLoadingZone1 = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(90))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90))
                .setTangent(FlipTangent(-90))
                .afterTime(.75, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(-90))
                .splineToSplineHeading(Launch, FlipTangent(-90))
                .waitSeconds(1)
                .build();

        Action fetchLoadingZone2 = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(90))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90))
                .setTangent(FlipTangent(-90))
                .afterTime(.75, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(-90))
                .splineToSplineHeading(Launch, FlipTangent(-90))
                .waitSeconds(1)
                .build();

        Action fetchLoadingZone3 = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(90))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90))
                .setTangent(FlipTangent(-90))
                .afterTime(.75, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(-90))
                .splineToSplineHeading(Launch, FlipTangent(-90))
                .waitSeconds(1)
                .build();

        Action fetchLoadingZone4 = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(90))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90))
                .setTangent(FlipTangent(-90))
                .afterTime(.75, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(-90))
                .build();

        Action driveToEnd = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(End, FlipTangent(180))
                .build();

        // Build the auto for launching preloads, fetching artifacts from the first spike and launching
        SequentialAction ret =  new SequentialAction(
                mTurret.GetDisableAction(true),
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherRunFast),
                    launchPreload
                ),
                mTurret.GetDisableAction(false),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mTurret.GetDisableAction(true),
                fetchLoadingZone1,
                mTurret.GetDisableAction(false),
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch)
        );
        // If more than one spike, add another fetch from the second spike and launch
        if (mSpikeCount > 1) {
            ret = new SequentialAction(
                ret,
                mTurret.GetDisableAction(true),
                fetchLoadingZone2,
                mTurret.GetDisableAction(false),
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch)
            );
        }
        // Add pickup of artifacts from final spike
        if (mSpikeCount > 2) {
            ret = new SequentialAction(
                ret,
                mTurret.GetDisableAction(true),
                fetchLoadingZone3,
                mTurret.GetDisableAction(false),
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch)
            );
        }
        if (pickupAtEnd) {
            ret = new SequentialAction(
                    ret,
                    mTurret.GetDisableAction(true),
                    fetchLoadingZone4,
                    mIntake.GetAction(IntakeActions.IntakeStop)
            );
        } else {
            ret = new SequentialAction(
                    ret,
                    mIntake.GetAction(IntakeActions.IntakeStop),
                    driveToEnd
            );
        }
        return ret;
    }
}
