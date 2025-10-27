package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Vision;
import org.firstinspires.ftc.teamcode.objects.LimelightVisionResult;
import org.firstinspires.ftc.teamcode.objects.VisionResult;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.VisionMode;

public class LimelightVision implements Vision {
    private Limelight3A ll;
    private LLResult result;
    private boolean isInit;

    public LimelightVision(HydraOpMode opMode) {
        ll = opMode.mHardwareMap.get(Limelight3A.class, "limelight");
        isInit = false;
    }

    @Override
    public boolean Init() {
        if (!isInit) {
            isInit = true;
            ll.setPollRateHz(Constants.LimelightPollRateHz);
            ll.start();
        }
        return true;
    }

    @Override
    public void Process() {
        result = ll.getLatestResult();
    }

    @Override
    public VisionResult GetResult() {
        if (result != null && result.isValid()) {
            return new LimelightVisionResult(result);
        } else {
            return null;
        }
    }

    @Override
    public void SetMode(VisionMode mode) {
        switch (mode) {
            case VisionMode_Obelisk:
                ll.pipelineSwitch(0);
                ll.start();
                break;
            case VisionMode_BlueGoal:
                ll.pipelineSwitch(1);
                ll.start();
                break;
            case VisionMode_RedGoal:
                ll.pipelineSwitch(2);
                ll.start();
                break;
            default:
                ll.pipelineSwitch(0);
                ll.pause();
                break;
        }
    }

    @Override
    public void HandleUserInput() {

    }
}
