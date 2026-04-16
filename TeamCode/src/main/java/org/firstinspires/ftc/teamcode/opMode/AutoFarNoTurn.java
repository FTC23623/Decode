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

public abstract class AutoFarNoTurn extends HydrAuto {
    public AutoFarNoTurn(VisionMode target, boolean flip, int spikeCount) {
        super(target, flip, spikeCount, 0);
        mBeginPose = FlipPose(64.0, 28.5, 90.0);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        //Pose2d Launch1 = FlipPose(55, 15, 90);
        Vector2d GPPPos = FlipCoordinate(28, 54);
        Vector2d PGPPos = FlipCoordinate(6, 48);
        Vector2d Launch2Pos = FlipCoordinate(63, 15);
        Pose2d GPP = new Pose2d(GPPPos, AutoTangent(Launch2Pos, GPPPos));
        Pose2d PGP = new Pose2d(PGPPos, AutoTangent(Launch2Pos, PGPPos));
        Pose2d Launch2 = new Pose2d(Launch2Pos, FlipTangent(90));
        Pose2d GPPSlowdownPose = Waypoint(Launch2, GPP, 0.75);
        Pose2d PGPSlowdownPose = Waypoint(Launch2, PGP, 0.75);

        double slowdownspeed = 20;

        //Action launchPreload = mDrive.actionBuilder(mBeginPose)
        //        .setTangent(FlipTangent(180))
        //        .splineToSplineHeading(Launch1, FlipTangent(180))
        //        .build();

        // Action to fetch artifacts from first spike and launch
        Action fetchGPP = mDrive.actionBuilder(Launch2)
                .setTangent(GPP.heading)
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(GPPSlowdownPose, GPP.heading)
                .splineToLinearHeading(GPP, GPP.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(AutoTangent(GPPPos, Launch2Pos))
                .splineToLinearHeading(Launch2, AutoTangent(GPPPos, Launch2Pos))
                .build();

        // Action to fetch artifacts from second spike and launch
        Action fetchPGP = mDrive.actionBuilder(Launch2)
                .setTangent(PGP.heading)
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                .splineToSplineHeading(PGPSlowdownPose, PGP.heading)
                .splineToSplineHeading(PGP, PGP.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(AutoTangent(PGPPos, Launch2Pos))
                .splineToLinearHeading(Launch2, AutoTangent(PGPPos, Launch2Pos))
//                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .build();

        // Launch preloads
        // Pickup from loading zone and launch
        // Pickup spike and launch
        SequentialAction ret =  new SequentialAction(
                mTurret.GetDisableAction(true),
                mTurret.GetSetAction(FlipTurret(-105)),
                new ParallelAction(
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherRunFast),
                    new SequentialAction(
          //                  launchPreload,
                            mTurret.GetDisableAction(false),
                            mTurret.GetLockAction()
                    )
                ),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                LoadingZoneSequence(Launch2, true, false, mBeginPose),
                mTurret.GetDisableAction(true),
                new ParallelAction(
                    fetchGPP,
                    mTurret.GetSetAction(FlipTurret(-105))
                ),
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
                new ParallelAction(
                    fetchPGP,
                    mTurret.GetSetAction(FlipTurret(-105))
                ),
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
                LoadingZoneSequence(Launch2, true, true, Launch2),
                LoadingZoneSequence(Launch2, true, true, Launch2)
            );
        }
        // for one or two spikes, pickup from loading zone at the end
        ret = new SequentialAction(
            ret,
            LoadingZoneSequence(Launch2, true, true, Launch2)
        );
        return ret;
    }
}
