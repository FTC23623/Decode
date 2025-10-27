package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.teamcode.types.DecodeAprilTag;
import java.util.List;

public class LimelightVisionResult implements VisionResult {
    private final LLResult result;

    public LimelightVisionResult(LLResult result) {
        this.result = result;
    }

    @Override
    public double GetXOffset() {
        return result.getTx();
    }

    @Override
    public double GetYOffset() {
        return result.getTy();
    }

    @Override
    public DecodeAprilTag GetTagClass() {
        List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
        if (!fid.isEmpty()) {
            return DecodeAprilTag.TagFromId(fid.get(0).getFiducialId());
        }
        return DecodeAprilTag.DecodeTag_Unknown;
    }

    @Override
    public long GetTimestamp() {
        return result.getControlHubTimeStamp();
    }
}
