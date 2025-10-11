package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.objects.Vision;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Drive_Manual;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Imu_Hub;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LimelightVision;
import org.firstinspires.ftc.teamcode.types.AprilTagClass;
import org.firstinspires.ftc.teamcode.types.VisionMode;

import java.util.ArrayList;
import java.util.List;

@Config
//@TeleOp(name = "HyDrive")
public class HyDrive extends LinearOpMode {
  private HydraOpMode mOpMode;
  private Imu mImu;
  private Drive mDrive;
  private Intake mIntake;
  private Vision mVision;
  private ElapsedTime mLoopSleep;
  private ArrayList<Subsystem> mSystems;
  protected final AprilTagClass tagClass = AprilTagClass.AprilTagClass_Unknown;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() throws InterruptedException {
    // Initialization Routines
    mLoopSleep = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    mOpMode = new HydraOpMode(telemetry, hardwareMap, gamepad1, gamepad2);
    mImu = new Imu_Hub(mOpMode);
    mDrive = new Drive_Manual(mOpMode, mImu);
    mIntake = new Intake(mOpMode);
    mVision = new LimelightVision(mOpMode);
    mOpMode.mVision = mVision;
    mSystems = new ArrayList<>();
    while (!mImu.Connected() || mImu.Calibrating()) {
      if (isStopRequested() || !opModeIsActive()) {
        break;
      }
    }
    mImu.SetYawOffset(OpmodeHeading.GetOffset());
    mSystems.add(mDrive);
    mSystems.add(mIntake);
    mSystems.add(mVision);
    telemetry.addData("Auto Yaw", OpmodeHeading.GetOffset());
    telemetry.update();
    // manual bulk caching
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    // intialize all subsystems
    for (Subsystem system : mSystems) {
      system.Init();
    }
    // set the vision up for targeting
    switch (tagClass) {
      case AprilTagClass_BlueGoal:
        mVision.SetMode(VisionMode.VisionMode_BlueGoal);
        break;
      case AprilTagClass_RedGoal:
        mVision.SetMode(VisionMode.VisionMode_RedGoal);
        break;
      default:
        mVision.SetMode(VisionMode.VisionMode_Disabled);
        break;
    }
    // wait for the operator to start the opmode
    waitForStart();
    mLoopSleep.reset();
    while (opModeIsActive()) {
      for (LynxModule module : allHubs) {
        module.clearBulkCache();
      }
      mOpMode.mLoopTime = mLoopSleep.milliseconds();
      // Pass user input to the systems
      for (Subsystem system : mSystems) {
        system.HandleUserInput();
      }
      // System processes
      for (Subsystem system : mSystems) {
        system.Process();
      }
      // Update telemetry once for all processes
      telemetry.update();
      mLoopSleep.reset();
      idle();
    }
  }
}