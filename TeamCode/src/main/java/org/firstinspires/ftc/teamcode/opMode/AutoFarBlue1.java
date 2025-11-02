package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueFar1", preselectTeleOp = "TeleBlue")
public class AutoFarBlue1 extends AutoFar {
    public AutoFarBlue1() {
        super(VisionMode.VisionMode_BlueGoal, true, 1);
    }
}
