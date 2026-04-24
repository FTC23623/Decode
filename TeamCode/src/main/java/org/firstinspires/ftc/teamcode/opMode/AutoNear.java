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

public abstract class AutoNear extends HydrAuto {
    public AutoNear(VisionMode target, boolean flip, int spikeCount) {
        super(target, flip, spikeCount, 0);
        mBeginPose = FlipPose(-62.5, 39, 0);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d Launch1 = FlipPose(-14, 20, 45);
        Vector2d PPGPos = FlipCoordinate(-12, 48);
        Vector2d PGPPos = FlipCoordinate(12, 50);
        Vector2d GPPPos = FlipCoordinate(36, 48);
        Pose2d Gate = FlipPose(2, 55, 90);
        Pose2d PPG = new Pose2d(PPGPos, AutoTangent(Launch1.position, PPGPos));
        Pose2d PGP = new Pose2d(PGPPos, AutoTangent(Launch1.position, PGPPos));
        Pose2d GPP = new Pose2d(GPPPos, AutoTangent(Launch1.position, GPPPos));
        Pose2d PPGSlowdownPose = Waypoint(Launch1, PPG, 0.75);
        Pose2d PGPSlowdownPose = Waypoint(Launch1, PGP, 0.75);
        Pose2d GPPSlowdownPose = Waypoint(Launch1, GPP, 0.75);
        Pose2d GateFeed = FlipPose(9, 60, 130);
        Pose2d GatePickup = FlipPose(13, 60, 130);

        double slowdownspeed = 20;
        double preloadtangent = AutoTangent(mBeginPose.position, Launch1.position);
        double fromgatetangent = AutoTangent(Gate.position, Launch1.position);
        double togatefeedtangent = AutoTangent(Launch1.position, GateFeed.position);
        double fromgatefeedtangent = AutoTangent(GateFeed.position, Launch1.position);

        // Action to fetch artifacts from first spike and launch
        Action launchPreload = mDrive.actionBuilder(mBeginPose)
                .setTangent(preloadtangent)
                .splineToLinearHeading(Launch1, preloadtangent)
                .build();

        Action fetchPPG = mDrive.actionBuilder(Launch1)
                .setTangent(PPG.heading)
                .splineToSplineHeading(PPGSlowdownPose, PPG.heading)
                .splineToSplineHeading(PPG, PPG.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(FlipTangent(0))
                .splineToLinearHeading(Gate, FlipTangent(90))
                .waitSeconds(0.5)
                .setTangent(fromgatetangent)
                .splineToLinearHeading(Launch1, fromgatetangent)
                .build();

        Action fetchPGP = mDrive.actionBuilder(Launch1)
                .setTangent(PGP.heading)
                .splineToSplineHeading(PGPSlowdownPose, PGP.heading)
                .splineToSplineHeading(PGP, PGP.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(FlipTangent(180))
                .splineToLinearHeading(Gate, FlipTangent(90))
                .waitSeconds(0.5)
                .setTangent(fromgatetangent)
                .splineToSplineHeading(Launch1, fromgatetangent)
                .build();

        Action fetchGPP = mDrive.actionBuilder(Launch1)
                .setTangent(GPP.heading)
                .splineToSplineHeading(GPPSlowdownPose, GPP.heading)
                .splineToSplineHeading(GPP, GPP.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(AutoTangent(GPPPos, Launch1.position))
                .splineToSplineHeading(Launch1, AutoTangent(GPPPos, Launch1.position))
                .build();

        Action gateFeed = mDrive.actionBuilder(Launch1)
                //.setTangent(FlipTangent(0))
                //.splineToLinearHeading(Gate, FlipTangent(90))
                .afterTime(0, mIntake.GetAction(IntakeActions.IntakeLoadArtifacts))
                //.setTangent(FlipTangent(-90))
                .setTangent(togatefeedtangent)
                .splineToLinearHeading(GateFeed, togatefeedtangent)
                .waitSeconds(0.1)
                .setTangent(FlipTangent(0))
                .splineToLinearHeading(GatePickup, FlipTangent(0))
                .waitSeconds(0.4)
                .setTangent(fromgatefeedtangent)
                .splineToLinearHeading(Launch1, fromgatefeedtangent)
                .build();

        Action park = mDrive.actionBuilder(Launch1)
                .setTangent(PPG.heading)
                .splineToLinearHeading(PPG, PPG.heading)
                .build();

        // Launch preloads
        // Pickup from loading zone and launch
        // Pickup spike and launch
        SequentialAction ret =  new SequentialAction(
                mTurret.GetDisableAction(true),
                mIntake.GetRejectDisableAction(true),
                new ParallelAction(
                    launchPreload,
                    mTurret.GetSetAction(FlipTurret(-80)),
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                    mLauncher.GetAction(LauncherActions.LauncherRunSlow)
                ),
                mTurret.GetDisableAction(false),
                mTurret.GetLockAction(),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetRejectDisableAction(false)
            );
        if (mSpikeCount > 2) {
            ret = new SequentialAction(
                ret,
                LaunchProcess(fetchPPG),
                LaunchProcess(fetchPGP),
                LaunchProcess(fetchGPP)
            );
        } else {
            ret = new SequentialAction(
                ret,
                LaunchProcess(fetchPGP),
                mTurret.GetDisableAction(true),
                gateFeed,
                mTurret.GetDisableAction(false),
                new ParallelAction(
                        mTurret.GetLockAction(),
                        mIntake.GetAction(IntakeActions.IntakePushToLauncher)
                ),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                LaunchProcess(fetchPPG)
            );
        }
        ret = new SequentialAction(
                ret,
                park
        );
        return ret;
    }

    SequentialAction LaunchProcess(Action drive) {
        return new SequentialAction(
            mTurret.GetDisableAction(true),
            mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
            drive,
            mTurret.GetDisableAction(false),
            new ParallelAction(
                    mTurret.GetLockAction(),
                    mIntake.GetAction(IntakeActions.IntakePushToLauncher)
            ),
            mLauncher.GetAction(LauncherActions.LauncherLaunch)
        );
    }
}
