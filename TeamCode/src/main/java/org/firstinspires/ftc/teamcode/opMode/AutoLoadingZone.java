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
                LoadingZoneSequence(Launch, true, false)
        );
        int count = mSpikeCount;
        while (count > 0) {
            count--;
            double waitTime = 0;
            if (count == 1) {
                waitTime = 0.25;
            }
            ret = new SequentialAction(
                    ret,
                    mDrive.actionBuilder(Launch).waitSeconds(waitTime).build(),
                    LoadingZoneSequence(Launch, count > 0, true)
            );
        }
        return ret;
    }
}
