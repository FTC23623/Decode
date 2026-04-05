package org.firstinspires.ftc.teamcode.types;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public final class Constants {
    public static final boolean fieldCentricDrive = true;
    public static final double trgBtnThresh = 0.05;
    public static final double driveBoosted = 1;
    public static final double driveNormal = 0.95;
    public static final double driveSlow = 0.25;
    public static final double contServoOff = 0.5;
    public static final double contServoForward = 1.0;
    public static final double contServoBackward = 0.05;
    public static final double joyStickDeadBand = 0.05;
    public static final int debounce = 3;
    public static final int debounceLong = 9;
    public static final double motorRpmIntervalMs = 50;
    public static final double linearLaunchMotTicksPerRev = 28;
    public static final double nsToMs = 1.0e-6;
    public static final double intakeMotorMaxIn = -1600000;
    public static final double intakeMotorMaxOut = 800000;
    public static final double LauncherTopRPMTele = 3025; //distance 135 - 3200, distance 120 - 2925, distance 125 - 3025
    public static final double LauncherTopRPMAuto = 2825;
    public static final double LauncherMedRPMThreshold = 2600;
    public static final double LauncherMedRPMTele = 2475;
    public static final double LauncherMedRPMAuto = 2450;
    // don't adjust low RPM above this threshold
    public static final double LauncherLowRPMThreshold = 2250;
    public static final double LauncherLowRPMTele = 2225;
    public static final double LauncherLowRPMAuto = 2175;
    public static final double LauncherIdleRPM = 500;
    public static final double LaunchServoRun = 1;
    public static  double TransfertoLaunchPower = 1600000; //ToDo: Find Value that works
    public static  double TransferFromIntakePower = 1600000;
    public static final double TransferToIntakePower = -500000;
    public static final double TransferMotorTickperRev = 103.8; // from goBilda for 1620rpm motor
    public static final double IntakeMotorTickperRev = 145.1 * 1.5; // from gobilda for 1150rpm Motor * 24/16 belt ratio
    //Turret Constants
    public static final double TurretGearRatioTurretToEncoder = 54.0/251; // ratio from Turret to Encoder 54/251
    public static final double TurretGearRatioTurretToServo = 0.9683642;// turret to servo 251/54*20/96
    public static double TurretAnalogEncoderBias = 7;//degrees Static offset between analog feedback and true zero
    public static final double TurretEncoderOffset = 180.0 / TurretGearRatioTurretToServo - TurretAnalogEncoderBias ; //Degrees ToDo: Set based on angle of Servo at zero Turret angle.

    public static double TurretMinPower = -0.2;
    public static double TurretMaxPower = 0.2; // Todo: Adjust based on tuning.
    public static double TurretDegreesPerTick = 360/8192.0 * TurretGearRatioTurretToEncoder; //CPR = 8192, encoder is on 54T side.
    public static double TurretMaxAngle = 118; //Degrees
    public static double TurretMinAngle = -118; //Degrees
    public static double TurretDeadbandDegrees = 1; // Tolerance wrt turret
    public static int TurretVisionLockTimeoutMs = 500;
    public static double TurretServoAnalogRangeVolts = 3.3;
    public static double DefaultVoltage = 12.7; // Default voltage use for controllers Voltage compensation

	//servo mode turret
    public static final double TurretRange = 355 / TurretGearRatioTurretToServo;// Servo Full Range / TurretGearRatioTurretToServo);

    // Vision and Tracking Constants
    // how many degrees back is your limelight rotated from perfectly vertical?
    public static double limelightMountAngleDegrees = 22;

    // distance from the center of the Limelight lens to the floor
    public static double limelightLensHeightInches = 9.5;

    // distance from the target to the floor
    public static double goalHeightInches = 38.5;
    public static int LimelightPollRateHz = 100;
    public static double LaunchWheelRpmDeadband = 50;
    public static double LauncherAutoLaunchTimeMs = 1300;
    public static int LauncherSpeedChangeWaitTimeMs = 6000;
    public static int IntakeRejectionTimeMs = 1000;
    public static int IntakeReversalTimeMs = 50;
    public static int ArtifactDetectionTimeMs = 100;

}
