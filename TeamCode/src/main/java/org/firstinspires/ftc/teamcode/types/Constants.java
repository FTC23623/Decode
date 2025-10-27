package org.firstinspires.ftc.teamcode.types;

public final class Constants {
    public static final boolean fieldCentricDrive = true;
    public static final double trgBtnThresh = 0.05;
    public static final double driveBoosted = 1;
    public static final double driveNormal = 0.95;
    public static final double driveSlow = 0.5;
    public static final double contServoOff = 0.5;
    public static final double contServoForward = 1.0;
    public static final double contServoBackward = 0.05;
    public static final double joyStickDeadBand = 0.05;
    public static final int debounce = 3;
    public static final int debounceLong = 9;
    public static final double motorRpmIntervalMs = 50;
    public static final double linearLaunchMotTicksPerRev = 28;
    public static final double nsToMs = 1.0e-6;
    public static final double intakeMotorMaxIn = -0.85;
    public static final double intakeMotorMaxOut = 1.0;
    public static final double LauncherTopRPM = 3050;
    public static final double LauncherTopRPMAuto = 2950;
    public static final double LauncherMedRPM = 2550;
    public static final double LauncherLowRPM = 2300;
    public static final double LauncherIdleRPM = 500;
    public static final double LaunchServoRun = 1;
    public static final double TransfertoLaunchPower = 0.8;
    public static final double TransferFromIntakePower = 0.8;
    public static final double TransferToIntakePower = -0.75;
    public static final double TurretGearRatio = 6.3;
    public static final double TurretRange = 355;
    // how many degrees back is your limelight rotated from perfectly vertical?
    public static double limelightMountAngleDegrees = -1.34;

    // distance from the center of the Limelight lens to the floor
    public static double limelightLensHeightInches = 4.5;

    // distance from the target to the floor
    public static double goalHeightInches = 9.5;
    public static int LimelightPollRateHz = 100;

}
