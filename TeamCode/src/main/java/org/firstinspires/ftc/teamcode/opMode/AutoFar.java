package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.types.IntakeActions;
import org.firstinspires.ftc.teamcode.types.LauncherActions;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoFar extends HydrAuto {
    public AutoFar(VisionMode target, boolean flip, int spikeCount) {
        super(target, flip, spikeCount, 0);
        mBeginPose = FlipPose(64.0, 28.5, 90.0);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        //Pose2d Launch1 = FlipPose(55, 15, 90);
        Vector2d GPPPos = FlipCoordinate(36, 48);
        Vector2d PGPPos = FlipCoordinate(12, 48);
        Vector2d Launch2Pos = FlipCoordinate(59, 21);
        Pose2d GPP = new Pose2d(GPPPos, AutoTangent(Launch2Pos, GPPPos));
        Pose2d PGP = new Pose2d(PGPPos, AutoTangent(Launch2Pos, PGPPos));
        Pose2d Launch2 = new Pose2d(Launch2Pos, FlipTangent(90));
        Pose2d GPPSlowdownPose = Waypoint(Launch2, GPP, 0.75);
        Pose2d PGPSlowdownPose = Waypoint(Launch2, PGP, 0.75);

        double slowdownspeed = 20;
        int lzcount = 0;

        // Action to fetch artifacts from first spike and launch
        Action fetchGPP = mDrive.actionBuilder(Launch2)
                .setTangent(GPP.heading)
                .splineToSplineHeading(GPPSlowdownPose, GPP.heading)
                .splineToSplineHeading(GPP, GPP.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(AutoTangent(GPPPos, Launch2Pos))
                .splineToSplineHeading(Launch2, AutoTangent(GPPPos, Launch2Pos))
                .build();

        // Action to fetch artifacts from second spike and launch
        Action fetchPGP = mDrive.actionBuilder(Launch2)
                .setTangent(PGP.heading)
                .splineToSplineHeading(PGPSlowdownPose, PGP.heading)
                .splineToSplineHeading(PGP, PGP.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(AutoTangent(PGPPos, Launch2Pos))
                .splineToSplineHeading(Launch2, AutoTangent(PGPPos, Launch2Pos))
                .build();

        // Launch preloads
        // Pickup from loading zone and launch
        // Pickup spike and launch
        SequentialAction ret =  new SequentialAction(
                mTurret.GetDisableAction(true),
                mIntake.GetRejectDisableAction(true),
                mTurret.GetSetAction(FlipTurret(-102)),
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherRunFast),
                    new SequentialAction(
                            mTurret.GetDisableAction(false),
                            mTurret.GetLockAction()
                    )
                ),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetRejectDisableAction(false),
                LoadingZoneSequence(Launch2, true, mBeginPose, false, lzcount++)
        );
        if (mSpikeCount > 0) {
            ret = new SequentialAction(
                ret,
                mTurret.GetDisableAction(true),
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                    fetchGPP,
                    mTurret.GetSetAction(FlipTurret(-109))
                ),
                mTurret.GetDisableAction(false),
                new ParallelAction(
                    mTurret.GetLockAction(),
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher)
                ),
                mLauncher.GetAction(LauncherActions.LauncherLaunch)
            );
        }
        // If more than one spike, add another fetch from the second spike and launch
        // Else pickup and launch from loading zone twice
        if (mSpikeCount > 1) {
            ret = new SequentialAction(
                ret,
                mTurret.GetDisableAction(true),
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                    fetchPGP,
                    mTurret.GetSetAction(FlipTurret(-109))
                ),
                mTurret.GetDisableAction(false),
                new ParallelAction(
                    mTurret.GetLockAction(),
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher)
                ),
                mLauncher.GetAction(LauncherActions.LauncherLaunch)
            );
        }

        int lzPickups = 5 - mSpikeCount;
        for (int i = 0; i < lzPickups; ++i) {
            ret = new SequentialAction(
                ret,
                LoadingZoneSequence(Launch2, true, Launch2, true, lzcount++)
            );
        }

        ret = new SequentialAction(
            ret,
            LoadingZoneSequence(Launch2, false, Launch2, true, 0)
        );
        return ret;
    }
}
