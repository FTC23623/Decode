package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueNear2", preselectTeleOp = "TeleBlue")
public class AutoNearBlue2 extends AutoNear {
    public AutoNearBlue2() {
        super(VisionMode.VisionMode_BlueGoal, true, 2);
    }
}
