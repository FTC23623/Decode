package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueLz3", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoLzBlue3 extends AutoLoadingZone {
    public AutoLzBlue3() {
        super(VisionMode.VisionMode_BlueGoal, true, 3);
    }
}
