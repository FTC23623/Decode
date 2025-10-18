package org.firstinspires.ftc.teamcode.opMode;

import static org.firstinspires.ftc.teamcode.types.VisionMode.VisionMode_Obelisk;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.Vision;
import org.firstinspires.ftc.teamcode.objects.VisionResult;
import org.firstinspires.ftc.teamcode.subsystems.Imu_Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.types.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.types.VisionMode;

import java.util.ArrayList;

public abstract class HydrAuto extends OpMode_Base {
    protected MecanumDrive mDrive;
    protected Intake mIntake;
    protected Pose2d mBeginPose;
    protected Vision mVision;
    protected Turret mTurret;
    protected ElapsedTime mTimeSinceStart;
    protected SequentialAction mAutoSeq;
    protected DecodeAprilTag mMotif;
    protected boolean mFlip;

    public HydrAuto(VisionMode tagToTarget, boolean flip) {
        super(tagToTarget);
        mFlip = flip;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        OpmodeHeading.handOff = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mOpMode = new HydraOpMode(telemetry, hardwareMap, null, null);
        mIntake = new Intake(mOpMode);
        mDrive = new MecanumDrive(hardwareMap, mBeginPose);
        mSystems = new ArrayList<>();
        mTurret = new Turret(mOpMode);
        mVision = new LimelightVision(mOpMode);
        mSystems.add(mIntake);
        mSystems.add(mVision);
        mSystems.add(mTurret);
        mOpMode.mVision = mVision;
        mTimeSinceStart = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        mAutoSeq = CreateAuto();
        SetLynxHubsManual();
        InitializeAllSystems();
        InitializeRRPinpoint();
        mVision.SetMode(VisionMode_Obelisk);
        mMotif = DecodeAprilTag.DecodeTag_Unknown;
        while (!isStarted() && !isStopRequested()) {
            ClearLynxHubCaches();
            mVision.Process();
            VisionResult result = mVision.GetResult();
            if (result != null) {
                DecodeAprilTag tag = result.GetTagClass();
                switch (tag) {
                    case DecodeTag_Obelisk_GPP:
                    case DecodeTag_Obelisk_PGP:
                    case DecodeTag_Obelisk_PPG:
                        if (tag != mMotif) {
                            mAutoSeq = CreateAuto();
                            mMotif = tag;
                        }
                        break;
                    default:
                        break;
                }
            }
            idle();
        }
        mTimeSinceStart.reset();
        mVision.SetMode(mVisionTarget);
        TelemetryPacket packet = new TelemetryPacket();
        mIntake.RunIn();
        while (opModeIsActive()) {
            ClearLynxHubCaches();
            for (Subsystem system : mSystems) {
                system.Process();
            }
            if(!mAutoSeq.run(packet)) {
                break;
            }
            telemetry.addLine(packet.toString());
            telemetry.update();
            idle();
        }
        mIntake.Stop();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
       // OpmodeHeading.SetOffset(mImu.GetYaw());
        OpmodeHeading.handOff = true;
    }
    
    protected void InitializeRRPinpoint() {
        /*
        if (mDrive.localizer instanceof PinpointLocalizer) {
            PinpointLocalizer ppl = (PinpointLocalizer) mDrive.localizer;
            while (!Imu_Pinpoint.InitPinpoint(ppl, mOpMode.mTelemetry)) {
                mOpMode.mTelemetry.update();
                mOpMode.mTelemetry.addLine("Initializing pinpoint...");
            }
        }*/
    }

    protected static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }

    protected SequentialAction CreateAuto() {
        return null;
    }

    protected Pose2d FlipPose(double x, double y, double headingDeg) {
        if (mFlip) {
            return new Pose2d(x, -y, HeadingRad(-headingDeg));
        } else {
            return new Pose2d(x, y, HeadingRad(headingDeg));
        }
    }

    protected Double FlipTangent(double degrees) {
        if (mFlip) {
            return Math.toRadians(-degrees);
        } else {
            return Math.toRadians(degrees);
        }
    }
}
