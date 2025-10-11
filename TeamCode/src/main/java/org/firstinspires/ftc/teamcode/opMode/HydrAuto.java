package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

public class HydrAuto extends LinearOpMode {
    protected HydraOpMode mOpMode;
    protected MecanumDrive mDrive;
    //protected Imu mImu;
    protected Intake mIntake;
    protected Pose2d mBeginPose;
    protected ElapsedTime mTimeSinceStart;
    protected SequentialAction mAutoSeq;
    protected ArrayList<Subsystem> mSystems;
    protected boolean mRunIntakeAtStart;

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
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        for (Subsystem system : mSystems) {
            system.Init();
        }
        waitForStart();
        mTimeSinceStart.reset();
        TelemetryPacket packet = new TelemetryPacket();
        mIntake.RunIn();
        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
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

    protected static double HeadingRad(double degrees) {
        return Math.toRadians(degrees);
    }

    protected SequentialAction CreateAuto() {
        return null;
    }
}
