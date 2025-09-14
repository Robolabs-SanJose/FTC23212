package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "TankDrive", group="Linear OpMode")

public class TankDrive extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    double axial;
    double yaw;

    @Override
    public void runOpMode(){
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive= hardwareMap.get(DcMotor.class, "right");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            axial = gamepad1.left_stick_y;
            yaw = gamepad1.right_stick_x;

            if (abs(yaw) > 0.1){
                leftDrive.setPower(yaw);
                rightDrive.setPower(-yaw);
            }else{
                leftDrive.setPower(axial);
                rightDrive.setPower(axial);
            }

            telemetry.addData("Status", "Runtime: "+ runtime.toString());
            telemetry.addData("","");
            telemetry.addData("Forward-Backward", axial);
            telemetry.addData("Rotation", yaw);
            telemetry.update();
        }
    }
}
