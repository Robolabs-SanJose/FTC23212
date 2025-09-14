package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MecanumOpMode", group="Linear OpMode")

public class RobotMovement extends LinearOpMode{
    private MecanumDrive ourRobot;
    double axial;
    double lateral;
    double yaw;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        ourRobot = new MecanumDrive("left_front_drive", "left_back_drive", "right_front_drive",
                "right_back_drive", 312.0, 1150.0, 435.0, 435.0);
        ourRobot.setMaxPower(0.5);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            axial = gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            ourRobot.updatePowerScaled(axial, lateral, yaw);
            ourRobot.updateMotors();
            ourRobot.publishTelemetry(axial,lateral, yaw);
        }
    }

    private class MecanumDrive{
        private DcMotor leftFrontDrive;
        private DcMotor leftBackDrive;
        private DcMotor rightFrontDrive;
        private DcMotor rightBackDrive;

        private double leftFrontPower;
        private double leftBackPower;
        private double rightFrontPower;
        private double rightBackPower;

        private double leftFrontRPM;
        private double leftBackRPM;
        private double rightFrontRPM;
        private double rightBackRPM;

        private double maxPower = 1.0;

        public MecanumDrive(String LFName, String LBName, String RFName, String RBName,
                            double LFRPM, double LBRPM, double RFRPM, double RBRPM){
            leftFrontDrive = hardwareMap.get(DcMotor.class, LFName);
            leftBackDrive = hardwareMap.get(DcMotor.class, LBName);
            rightFrontDrive = hardwareMap.get(DcMotor.class, RFName);
            rightBackDrive = hardwareMap.get(DcMotor.class, RBName);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            leftFrontRPM = LFRPM;
            leftBackRPM = LBRPM;
            rightFrontRPM = RFRPM;
            rightBackRPM = RBRPM;
        }

        public void updatePowerSimple(double axial, double lateral, double yaw){
            double savedMaxPower = maxPower;
            setMaxPower(1.0);
            updatePowerScaled(axial, lateral, yaw);
            setMaxPower(maxPower);
        }

        public void updatePowerScaled(double axial, double lateral, double yaw) {
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            normalizePowers();
        }

        public void setMaxPower(double newMaxPower){
            if (abs(newMaxPower) <= 1) {
                maxPower = newMaxPower;
            }
        }

        public double[] updatePowerSquareInputs(double axial, double lateral, double yaw){
            double highestInput;

            highestInput = max(abs(axial), abs(lateral));
            highestInput = max(highestInput, abs(yaw));
            highestInput *= highestInput;

            axial *= highestInput;
            lateral *= highestInput;
            yaw *= highestInput;

            return new double[] {axial, lateral, yaw};
        }

        public void updatePowerSquared(double axial, double lateral, double yaw) {
            double[] directions = updatePowerSquareInputs(axial, lateral, yaw);
            updatePowerScaled(directions[0], directions[1], directions[2]);
        }

        public void normalizePowers(){
            double max;

            max = max(max(abs(leftFrontPower), abs(rightFrontPower)), max(abs(leftBackPower), abs(rightBackPower)));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontPower *= maxPower;
            rightFrontPower *= maxPower;
            leftBackPower *= maxPower;
            rightBackPower *= maxPower;
        }

        public void updateMotors(){
            double minRPM = min(min(leftFrontRPM, leftBackRPM), min(rightFrontRPM, rightBackRPM));

            leftFrontDrive.setPower(leftFrontPower * (minRPM / leftFrontRPM));
            leftBackDrive.setPower(leftBackPower * (minRPM / leftBackRPM));
            rightFrontDrive.setPower(rightFrontPower * (minRPM / rightFrontRPM));
            rightBackDrive.setPower(rightBackPower * (minRPM / rightBackRPM));
        }

        public void publishTelemetry(double axial, double lateral, double yaw){
            telemetry.addData("run time:", runtime);
            telemetry.addData("","");
            telemetry.addData("forward-backward", axial);
            telemetry.addData("left-right", lateral);
            telemetry.addData("rotation", yaw);
            telemetry.addData("","");
            telemetry.addData("Left Front Power",leftFrontPower);
            telemetry.addData("Left Back Power",leftBackPower);
            telemetry.addData("Right Front Power", rightFrontPower);
            telemetry.addData("Right Back Power", rightBackPower);
            telemetry.update();
        }
    }
}