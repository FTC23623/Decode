package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;

import java.util.ArrayList;

@Config
public class Lift implements Subsystem {
    private final HydraOpMode opMode;
    private final Debouncer driver;
    private final Debouncer operator;
    private final ArrayList<Servo> servos;
    private boolean engaged;
    public static double engagedPos = 0.71;
    public static double disengagedPos = 0.23;

    public Lift(HydraOpMode opMode) {
        this.opMode = opMode;
        driver = new Debouncer(2);
        operator = new Debouncer(2);
        servos = new ArrayList<>(2);
        Servo LiftLeft = opMode.mHardwareMap.get(Servo.class, "liftLeft");
        Servo LiftRight = opMode.mHardwareMap.get(Servo.class, "liftRight");
        LiftLeft.setDirection(Servo.Direction.REVERSE);
        LiftRight.setDirection(Servo.Direction.FORWARD);
        servos.add(LiftLeft);
        servos.add(LiftRight);
        engaged = false;
    }

    @Override
    public boolean Init() {
        for (Servo servo : servos) {
            servo.setPosition(disengagedPos);
        }
        return true;
    }

    @Override
    public void Process() {
        if (driver.Out() && operator.Out()) {
            driver.Used();
            operator.Used();
            double position;
            if (engaged) {
                position = disengagedPos;
                engaged = false;
            } else {
                position = engagedPos;
                engaged = true;
            }
            for (Servo servo : servos) {
                servo.setPosition(position);
            }
        }
        opMode.mTelemetry.addData("LiftEngaged", engaged);
    }

    @Override
    public void HandleUserInput() {
        driver.In(opMode.mDriverGamepad.cross);
        operator.In(opMode.mOperatorGamepad.cross);
    }

    public boolean Engaged() {
        return engaged;
    }
}
