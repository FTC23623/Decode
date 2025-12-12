package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.types.IntakeActions;
import org.firstinspires.ftc.teamcode.types.LauncherActions;
import org.firstinspires.ftc.teamcode.types.TurretActions;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoFar extends HydrAuto {
    public AutoFar(VisionMode target, boolean flip, int spikeCount) {
        super(target, flip, spikeCount, 0);
        mBeginPose = FlipPose(60.0, 15.0, 0.0);
    }

    @Override
    protected SequentialAction CreateAuto() {

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

        Action launchPreload = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(Launch, FlipTangent(180))
                .build();

        // Action to fetch artifacts from first spike and launch
        Action fetchGPP = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(180))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(GPP_WP, FlipTangent(90))
                .setTangent(FlipTangent(90))
                .splineToSplineHeading(GPP, FlipTangent(90))
                .setTangent(FlipTangent(-90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToLinearHeading(Launch, FlipTangent(-90))
                .build();

        // Action to fetch artifacts from second spike and launch
        Action fetchPGP = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(180))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(PGP_WP, FlipTangent(90))
                .setTangent(FlipTangent(90))
                .splineToSplineHeading(PGP, FlipTangent(90))
                .setTangent(FlipTangent(-90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .splineToLinearHeading(Launch, FlipTangent(-60))
                .build();

        // Launch preloads
        // Pickup from loading zone and launch
        // Pickup spike and launch
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
                LoadingZoneSequence(Launch, true, false),
                mTurret.GetDisableAction(true),
                fetchGPP,
                mTurret.GetDisableAction(false),
                new ParallelAction(
                    mTurret.GetLockAction(),
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher)
                ),
                mLauncher.GetAction(LauncherActions.LauncherLaunch)
        );
        // If more than one spike, add another fetch from the second spike and launch
        // Else pickup and launch from loading zone twice
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
        } else {
            ret = new SequentialAction(
                ret,
                LoadingZoneSequence(Launch, true, true),
                LoadingZoneSequence(Launch, true, true)
            );
        }
        // for one or two spikes, pickup from loading zone at the end
        ret = new SequentialAction(
            ret,
            LoadingZoneSequence(Launch, false, true)
        );
        return ret;
    }
}
