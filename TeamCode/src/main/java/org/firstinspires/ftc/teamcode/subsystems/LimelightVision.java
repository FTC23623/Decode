package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Vision;
import org.firstinspires.ftc.teamcode.objects.LimelightVisionResult;
import org.firstinspires.ftc.teamcode.objects.VisionResult;
import org.firstinspires.ftc.teamcode.types.Constants;
import org.firstinspires.ftc.teamcode.types.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.types.VisionMode;
import java.util.ArrayList;

public class LimelightVision implements Vision {
    private Limelight3A ll;
    private LLResult result;
    private boolean isInit;
    private ArrayList<DecodeAprilTag> targetTags;

    public LimelightVision(HydraOpMode opMode) {
        ll = opMode.mHardwareMap.get(Limelight3A.class, "limelight");
        isInit = false;
        targetTags = new ArrayList<>();
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
            VisionResult ret = new LimelightVisionResult(result);
            if (targetTags.contains(ret.GetTagClass())) {
                return ret;
            } else {
                return null;
            }
        } else {
            return null;
        }
    }

    @Override
    public void SetMode(VisionMode mode) {
        targetTags.clear();
        result = null;
        switch (mode) {
            case VisionMode_Obelisk:
                targetTags.add(DecodeAprilTag.DecodeTag_Obelisk_GPP);
                targetTags.add(DecodeAprilTag.DecodeTag_Obelisk_PGP);
                targetTags.add(DecodeAprilTag.DecodeTag_Obelisk_PPG);
                ll.pipelineSwitch(0);
                ll.start();
                break;
            case VisionMode_BlueGoal:
                targetTags.add(DecodeAprilTag.DecodeTag_BlueGoal);
                ll.pipelineSwitch(1);
                ll.start();
                break;
            case VisionMode_RedGoal:
                targetTags.add(DecodeAprilTag.DecodeTag_RedGoal);
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
