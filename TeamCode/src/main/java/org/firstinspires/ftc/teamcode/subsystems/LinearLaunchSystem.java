package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.LaunchMotor;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LinearLaunchSystem {
    private final LaunchMotor left;
    private final LaunchMotor right;

    public LinearLaunchSystem(HydraOpMode Opmode){
        left =Opmode.mHardwareMap.get(DcMotorEx.class, "left");
        right = Opmode.mHardwareMap.get(DcMotorEx.class, "right");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDcoeffRight = new PIDFCoefficients();
        PIDcoeffLeft = new PIDFCoefficients();
        PIDleft=new PIDFController(PIDcoeffLeft.p, PIDcoeffLeft.i, PIDcoeffLeft.d, PIDcoeffLeft.f);
        PIDright = new PIDFController(PIDcoeffRight.p, PIDcoeffRight.i, PIDcoeffRight.d, PIDcoeffRight.f);
    }
    void process() {
        double leftPower=0;
        double rightPower=0;
        double leftrpm=getrpm(left);
        double rightrpm=getrpm(right);
        leftPower=PIDleft.calculate(leftrpm);
        rightPower=PIDright.calculate(rightrpm);
        left.setPower(leftPower);
        right.setPower(rightPower);

    }
    double getrpm(DcMotorEx motor){
        double position= motor.getCurrentPosition();
        double dif= position-lastposition;
        double timenow=time.now(ElapsedTime.Resolution.MILLISECONDS);
        double timedif=timenow-lasttime;
        double rev=dif/ticksperrev;
        lastposition=position;
        lasttime=timenow;
        return rev/timedif;

    }
}
