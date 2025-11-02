package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "RedFar2", preselectTeleOp = "TeleRed")
public class AutoFarRed2 extends AutoFar {
    public AutoFarRed2() {
        super(VisionMode.VisionMode_RedGoal, false, 2);
    }
}
