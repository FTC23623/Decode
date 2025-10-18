package org.firstinspires.ftc.teamcode.opMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.types.VisionMode;

import java.util.ArrayList;
import java.util.List;

public abstract class OpMode_Base extends LinearOpMode {

    protected HydraOpMode mOpMode;
    protected List<LynxModule> allHubs;
    protected ArrayList<Subsystem> mSystems;
    protected VisionMode mVisionTarget;

    public OpMode_Base(VisionMode target) {
        mVisionTarget = target;
    }

    protected void SetLynxHubsManual() {
        allHubs = mOpMode.mHardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    protected void ClearLynxHubCaches() {
      for (LynxModule module : allHubs) {
          module.clearBulkCache();
      }
    }

    protected void InitializeAllSystems() {
        boolean init = true;
        do {
            ClearLynxHubCaches();
            for (Subsystem system : mSystems) {
                init &= system.Init();
            }
            mOpMode.mTelemetry.update();
        } while (!init);
    }
}
