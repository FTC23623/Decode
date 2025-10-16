package org.firstinspires.ftc.teamcode.opMode;

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
import org.firstinspires.ftc.teamcode.subsystems.Imu_Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;

public class HydrAuto extends OpMode_Base {
    protected MecanumDrive mDrive;
    protected Intake mIntake;
    protected Pose2d mBeginPose;
    protected ElapsedTime mTimeSinceStart;
    protected SequentialAction mAutoSeq;

    @Override
    public void runOpMode() throws InterruptedException {
        OpmodeHeading.handOff = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mOpMode = new HydraOpMode(telemetry, hardwareMap, null, null);
        mIntake = new Intake(mOpMode);
        mDrive = new MecanumDrive(hardwareMap, mBeginPose);
        mSystems = new ArrayList<>();
        mSystems.add(mIntake);
        mTimeSinceStart = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        mAutoSeq = CreateAuto();
        SetLynxHubsManual();
        InitializeAllSystems();
        InitializeRRPinpoint();
        waitForStart();
        mTimeSinceStart.reset();
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
    
    private void InitializeRRPinpoint() {
        if (mDrive.localizer instanceof PinpointLocalizer) {
            PinpointLocalizer ppl = (PinpointLocalizer) mDrive.localizer;
            while (!Imu_Pinpoint.InitPinpoint(ppl, mOpMode.mTelemetry)) {
                mOpMode.mTelemetry.update();
                mOpMode.mTelemetry.addLine("Initializing pinpoint...");
            }
        }
    }

    protected static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }

    protected SequentialAction CreateAuto() {
        return null;
    }
}
