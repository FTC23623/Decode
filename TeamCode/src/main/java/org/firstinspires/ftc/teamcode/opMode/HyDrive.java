package org.firstinspires.ftc.teamcode.opMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.types.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.types.VisionMode;

import java.util.ArrayList;

@Config
//@TeleOp(name = "HyDrive")
public abstract class HyDrive extends OpMode_Base {
  private Imu mImu;
  private Drive mDrive;
  private Intake mIntake;
  private Vision mVision;
  private ElapsedTime mLoopSleep;
  protected DecodeAprilTag tagClass;

  public HyDrive(DecodeAprilTag tagClass) {
    this.tagClass = tagClass;
  }

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
    mSystems.add(mDrive);
    mSystems.add(mIntake);
    mSystems.add(mVision);
    mSystems.add(mImu);
    // manual bulk caching
    SetLynxHubsManual();
    InitializeAllSystems();
    mImu.SetYawOffset(OpmodeHeading.GetOffset());
    telemetry.addData("Auto Yaw", OpmodeHeading.GetOffset());
    telemetry.update();
    // set the vision up for targeting
    switch (tagClass) {
      case DecodeTag_BlueGoal:
        mVision.SetMode(VisionMode.VisionMode_BlueGoal);
        break;
      case DecodeTag_RedGoal:
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
      ClearLynxHubCaches();
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
