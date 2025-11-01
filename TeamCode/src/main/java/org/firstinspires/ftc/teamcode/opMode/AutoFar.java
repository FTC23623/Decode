package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.types.IntakeActions;
import org.firstinspires.ftc.teamcode.types.LauncherActions;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoFar extends HydrAuto {
    public AutoFar(VisionMode target, boolean flip) {
        super(target, flip);
        mBeginPose = FlipPose(60.0, 15.0, 0.0);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d Launch = FlipPose(55, 15, 0);
        Pose2d GPP_WP = FlipPose(34, 30, 90);
        Pose2d GPP = FlipPose(44, 51, 90);
        Pose2d PGP_WP = FlipPose(11, 30, 90);
        Pose2d PGP = FlipPose(21, 51, 90);
        Pose2d PPG_WP = FlipPose(-12, 35, 90);
        Pose2d PPG = FlipPose(-12, 48, 90);
        Pose2d End = FlipPose(30, 15, 0);

        Action fetchGPP = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(GPP_WP, FlipTangent(90))
                .setTangent(FlipTangent(90))
                .splineToSplineHeading(GPP, FlipTangent(0))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .afterTime(1.5, mIntake.GetAction(IntakeActions.IntakeStop))
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(Launch, FlipTangent(-90))
                .waitSeconds(.75)
                .build();

        Action fetchPGP = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(PGP_WP, FlipTangent(90))
                .setTangent(FlipTangent(90))
                .splineToSplineHeading(PGP, FlipTangent(0))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .afterTime(1.5, mIntake.GetAction(IntakeActions.IntakeStop))
                .setTangent(FlipTangent(0))
                .splineToSplineHeading(Launch, FlipTangent(-90))
                .waitSeconds(.75)
                .build();

        Action pickupPPG = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(PPG_WP, FlipTangent(90))
                .setTangent(FlipTangent(90))
                .splineToSplineHeading(PPG, FlipTangent(90))
                .afterTime(1, mIntake.GetAction(IntakeActions.IntakeReject))
                .afterTime(1.5, mIntake.GetAction(IntakeActions.IntakeStop))
                //.setTangent(FlipTangent(0))
                //.splineToSplineHeading(Launch, FlipTangent(-90))
                //.waitSeconds(.75)
                .build();

        Action driveToEnd = mDrive.actionBuilder(Launch)
                .setTangent(FlipTangent(180))
                .splineToSplineHeading(End, FlipTangent(180))
                .build();

        return new SequentialAction(
                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mTurret.GetSetAction(FlipTurret(-20)),
                mLauncher.GetAction(LauncherActions.LauncherRunFast),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),

                mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                fetchGPP,

                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),

                mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                fetchPGP,

                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),

                mIntake.GetAction(IntakeActions.IntakeLoadArtifacts),
                pickupPPG/*,

                mIntake.GetAction(IntakeActions.IntakePushToLauncher),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                mIntake.GetAction(IntakeActions.IntakeStop),
                driveToEnd*/
                );
    }
}
