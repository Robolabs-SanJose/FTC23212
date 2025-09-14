package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name =  "arm", group =  "linear opmode")

public class Arm extends FeedForwardController{
    private final DcMotor Motor;
    public Arm(OpMode robot, String MotorName){
        Motor = robot.hardwareMap.get(DcMotor.class, MotorName);
    }

    public double update(double angle){
        return ks * sin(angle) + kv * speed + ka * acceleration;
    }

    public void updateMotor(double angle){
        Motor.setPower(update(angle));
    }
}
