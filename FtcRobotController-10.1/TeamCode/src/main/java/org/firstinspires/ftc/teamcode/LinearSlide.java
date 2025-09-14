package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static java.lang.Math.abs;

public class LinearSlide {
    public DcMotor Motor1 = null;
    public DcMotor Motor2 = null;
    PIDController PID = null;

    public boolean up = false;
    public double runtime = 0.0;
    public final float ticksPerRotation = 1f;// set later; f declares as a float
    public final float rotationsPerLift = 1f;// set later; f declares as a float


    public LinearSlide(OpMode robot, String Motor1Name, String Motor2Name, double k_p, double k_i, double k_d) {
        PID =  new PIDController(k_p, k_i, k_d);
        Motor1 = robot.hardwareMap.get(DcMotor.class, Motor1Name);
        Motor2 = robot.hardwareMap.get(DcMotor.class, Motor2Name);
        PID.setSetpoint(0);
    }

    public void changeState(){
        if (up) {
            up = false;
            PID.setSetpoint(rotationsPerLift);
        }else if (runtime > 90){
            up = true;
            PID.setSetpoint(0);
        }
    }

    public void update(double runtime){
        this.runtime = runtime;
        double current = (Motor1.getCurrentPosition() + Motor2.getCurrentPosition()) / 2;
        current /= ticksPerRotation;
        double power = PID.update(current,false);

        if (!up && abs(power) > .25){
            power = -.25;
        }
        
        Motor1.setPower(power);
        Motor2.setPower(power);
    }
}