package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueNear1", preselectTeleOp = "TeleBlue")
public class AutoNearBlue1 extends AutoNear {
    public AutoNearBlue1() {
        super(VisionMode.VisionMode_BlueGoal, true, 1);
    }
}
