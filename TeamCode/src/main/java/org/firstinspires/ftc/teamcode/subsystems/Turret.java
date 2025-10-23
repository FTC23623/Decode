package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
@Config
public class Turret implements Subsystem {
    private final HydraOpMode mOp;
    public static double mPosChangeRate = 0.15;
    public static double mMaxPos = 1;
    public static double mMinPos = 0;
    private double UserInput = 0;
    private Servo TurretServo;

    public Turret(HydraOpMode opMode) {
        mOp = opMode;
        TurretServo = mOp.mHardwareMap.get(Servo.class,"TurretServo");
        TurretServo.setPosition(0.5);
    }

    @Override
    public boolean Init() {
        return true;
    }

    @Override
    public void HandleUserInput() {
        UserInput = mOp.mOperatorGamepad.right_stick_x;
    }

    @Override
    public void Process() {
        // scale user input with a constant rate
        double position_change = UserInput * mPosChangeRate;
        // get the last set position and calculate the new position
        double CurrentPos = TurretServo.getPosition();
        double NewPos = CurrentPos + position_change;
        // clamp the new position to the min and max
        NewPos = Math.min(mMaxPos, NewPos);
        NewPos = Math.max(mMinPos, NewPos);
        // set the new position
        TurretServo.setPosition(NewPos);
        mOp.mTelemetry.addData("Turret Position", NewPos);
    }
}
