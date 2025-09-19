package org.firstinspires.ftc.teamcode.opModes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.HydraSubsystem;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Drive_Manual;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Imu_Hub;

import java.util.ArrayList;
import java.util.List;

@Config
@Disabled
@TeleOp(name = "HyDrive")
public class HyDrive extends LinearOpMode {
  private HydraOpMode mOpMode;
  private Imu mImu;
  private Drive mDrive;
  private ElapsedTime mLoopSleep;
  private ArrayList<HydraSubsystem> mSystems;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    // Initialization Routines
    mLoopSleep = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    mOpMode = new HydraOpMode(telemetry, hardwareMap, null, null, gamepad1,
            gamepad2);
    mImu = new Imu_Hub(mOpMode);
    mDrive = new Drive_Manual(mOpMode, mImu);
    while (!mImu.Connected() || mImu.Calibrating()) {
      if (isStopRequested() || !opModeIsActive()) {
        break;
      }
    }
    mImu.SetYawOffset(OpmodeHeading.GetOffset());
    mSystems.add(mDrive);
    telemetry.addData("Auto Yaw", OpmodeHeading.GetOffset());
    telemetry.update();
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    waitForStart();
    mLoopSleep.reset();
    while (opModeIsActive()) {
      for (LynxModule module : allHubs) {
        module.clearBulkCache();
      }
      mOpMode.mLoopTime = mLoopSleep.milliseconds();
      mLoopSleep.reset();
      idle();
    }
    mLoopSleep.reset();
    while (opModeIsActive()) {
      for (LynxModule module : allHubs) {
        module.clearBulkCache();
      }
      mOpMode.mLoopTime = mLoopSleep.milliseconds();
      // Pass user input to the systems
      // System processes
      for (HydraSubsystem system : mSystems) {
        system.Process();
      }
      // Update telemetry once for all processes
      telemetry.update();
      mLoopSleep.reset();
      idle();
    }
  }
}