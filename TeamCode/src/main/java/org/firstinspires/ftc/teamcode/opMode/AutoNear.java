package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.types.LauncherActions;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public abstract class AutoNear extends HydrAuto {

    public AutoNear(VisionMode target, boolean flip) {
        super(target, flip);
        mBeginPose = FlipPose(-54, 52, 310);
    }

    @Override
    protected SequentialAction CreateAuto() {

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d GPP_WP = FlipPose(46, 30, 90);
        Pose2d GPP = FlipPose(46, 54, 90);
        Pose2d PGP_WP = FlipPose(22, 30, 90);
        Pose2d PGP = FlipPose(22, 54, 90);
        Pose2d PPG_WP = FlipPose(-4, 30, 90);
        Pose2d PPG = FlipPose(-4, 54, 90);
        Pose2d LaunchNear = FlipPose(-25, 24, -40);
        Pose2d End = FlipPose(10, 20, 0);

        Action driveToLaunch1 = mDrive.actionBuilder(mBeginPose)
                .setTangent(FlipTangent(315))
                .splineToSplineHeading(LaunchNear, FlipTangent(315))
                .waitSeconds(.75)
                .build();

        Action driveToLaunch2 = mDrive.actionBuilder(LaunchNear)
                .splineToSplineHeading(PPG_WP, FlipTangent(90))
                .splineToSplineHeading(PPG, FlipTangent(90))
                .setTangent(FlipTangent(225))
                .splineToSplineHeading(LaunchNear, FlipTangent(-90))
                .waitSeconds(.75)
                .build();

        Action driveToLaunch3 = mDrive.actionBuilder(LaunchNear)
                .splineToSplineHeading(PGP_WP, FlipTangent(90))
                .splineToSplineHeading(PGP, FlipTangent(90))
                .setTangent(FlipTangent(225))
                .splineToSplineHeading(LaunchNear, FlipTangent(225))
                .waitSeconds(.75)
                .build();

        Action driveToLaunch4 = mDrive.actionBuilder(LaunchNear)
                .setTangent(FlipTangent(25))
                .splineToSplineHeading(GPP_WP, FlipTangent(90))
                .splineToSplineHeading(GPP, FlipTangent(90))
                .setTangent(FlipTangent(225))
                .splineToSplineHeading(LaunchNear, FlipTangent(180))
                .waitSeconds(.75)
                .build();

        Action driveToEnd = mDrive.actionBuilder(LaunchNear)
                .splineToSplineHeading(End, FlipTangent(0))
                .build();

        return new SequentialAction(
                new ParallelAction(
                    mLauncher.GetAction(LauncherActions.LauncherRunSlow),
                    driveToLaunch1),
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                driveToLaunch2,
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                driveToLaunch3,
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                driveToLaunch4,
                mLauncher.GetAction(LauncherActions.LauncherLaunch),
                driveToEnd);
    }
}
