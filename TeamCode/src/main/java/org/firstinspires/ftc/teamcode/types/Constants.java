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
    public static final double intakeMotorMaxIn = -0.85;
    public static final double intakeMotorMaxOut = .6;
    public static final double LauncherTopRPMTele = 2925;
    public static final double LauncherTopRPMAuto = 2825;
    public static final double LauncherMedRPMThreshold = 2600;
    public static final double LauncherMedRPMTele = 2425;
    public static final double LauncherMedRPMAuto = 2450;
    // don't adjust low RPM above this threshold
    public static final double LauncherLowRPMThreshold = 2250;
    public static final double LauncherLowRPMTele = 2175;
    public static final double LauncherLowRPMAuto = 2175;
    public static final double LauncherIdleRPM = 500;
    public static final double LaunchServoRun = 1;
    public static final double TransfertoLaunchPower = .95;
    public static final double TransferFromIntakePower = .95;
    public static final double TransferToIntakePower = -0.75;

    //Turret Constants
    public static final double TurretGearRatioTurretToEncoder = 1/3.15; // ratio from Turret to Encoder 80/252
    public static final double TurretGearRatioTurretToServo = 0.65625;// turret to servo 252/80*20/96
    //Set based Offset angle of Axon feedback at turret zero position. Note: Turret Zero should be set close to 180deg so the +/- rotation does not go through Axon 0 deg.
    public static final double TurretEncoderOffset = 180.0 / TurretGearRatioTurretToServo ; //Degrees ToDo: Set based on angle of Servo at zero Turret angle.
    public static PIDFCoefficients TurretPIDFCoefficients = new PIDFCoefficients(0.0, 0.0, 0.0,0.0); //ToDo Set Turret PIDF Coefficients based on Units and Tuning.
    public static double TurretFF = 0.00; // 0.029 Power Acts as feedforward term when turret PIDF is non zero
    public static double TurretMinPower = 0;
    public static double TurretMaxPower = 0.5; // Todo: Adjust based on tuning.
    public static double TurretDegreesPerTick = 360/8192.0 * TurretGearRatioTurretToEncoder; //CPR = 8192, encoder is on 80T side. Todo: Check Math
    public static double TurretMaxAngle = 100; //Degrees
    public static double TurretMinAngle = -100; //Degrees
    public static double TurretDeadbandDegrees = 1; // Tolerance wrt turret
    public static int TurretVisionLockTimeoutMs = 500;
    public static double TurretServoAnalogRangeVolts = 3.3;
    public static double DefaultVoltage = 12.7; // Default voltage use for controllers Voltage compensation

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
