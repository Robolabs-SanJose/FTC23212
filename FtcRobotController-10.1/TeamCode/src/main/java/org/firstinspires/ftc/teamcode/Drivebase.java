package org.firstinspires.ftc.teamcode;

import static java.lang.Math.max;
import static java.lang.Math.abs;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivebase {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor rightFrontDrive;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    double axial;
    double lateral;
    double yaw;
    private static boolean initialized = false;
    private OpMode robot;
    IMU imu;
    private PIDController turnController;

    private Drivebase(){}
    private static final Drivebase INSTANCE = new Drivebase();
    public static Drivebase getInstance() {return INSTANCE;}

    public void initialize(OpMode robot, String leftFrontName, String leftBackName,
                           String rightFrontName, String rightBackName, RevHubOrientationOnRobot orientationOnRobot) {
        if (initialized) {
            return;
        }

        this.robot = robot;

        leftFrontDrive = robot.hardwareMap.get(DcMotor.class, leftFrontName);
        leftBackDrive = robot.hardwareMap.get(DcMotor.class, leftBackName);
        rightFrontDrive = robot.hardwareMap.get(DcMotor.class, rightFrontName);
        rightBackDrive = robot.hardwareMap.get(DcMotor.class, rightBackName);
        imu = robot.hardwareMap.get(IMU.class,"imu")
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        turnController = new PIDController(.01, 0, 0);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        runtime.reset();
        initialized = true;
    }

    public static void reset(){initialized = false;}

    public void update(double axial, double lateral, double turningX, double turningY) {
        this.axial = axial;
        this.lateral = lateral;

        YawPitchRollAngles imuAngles = imu.getRobotYawPitchRollAngles();
        double imuData = imuAngles.getYaw(AngleUnit.DEGREES);
        imuData = -imuData;
        if (imuData < 0) {
            imuData += 360;
        }

        if ((turningX != 0) && (turningY != 0)){
            double heading = atan2(turningX, -turningY);
            heading *= 180/PI;
            turnController.setSetpoint(heading);
        }

        yaw = turnController.update(imuData, true)

        leftFrontPower = axial + lateral - yaw;
        rightFrontPower = axial - lateral + yaw;
        leftBackPower = axial - lateral - yaw;
        rightBackPower = axial + lateral + yaw;

        double max = max(abs(leftFrontPower), abs(rightFrontPower));
        max = max(max, abs(leftBackPower));
        max = max(max, abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void updateTelemetry(YawPitchRollAngles orientation, AngularVelocity angularVelocity, PIDController PID){
        robot.telemetry.addData("run time:", runtime);
        robot.telemetry.addData("","");
        robot.telemetry.addData("forward-backward", axial);
        robot.telemetry.addData("left-right", lateral);
        robot.telemetry.addData("rotation", yaw);
        robot.telemetry.addData("","");
        robot.telemetry.addData("Left Front Power",leftFrontPower);
        robot.telemetry.addData("Left Back Power",leftBackPower);
        robot.telemetry.addData("Right Front Power", rightFrontPower);
        robot.telemetry.addData("Right Back Power", rightBackPower);
        robot.telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        robot.telemetry.addData("Pitch (Y)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        robot.telemetry.addData("Roll (X)", "%.2f Deg.", orientation.getRoll(AngleUnit.DEGREES));
        robot.telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        robot.telemetry.addData("Pitch (Y) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        robot.telemetry.addData("Roll (X) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
        robot.telemetry.addData("","");
        robot.telemetry.addData("PID set point", PID.getSetpoint());
        robot.telemetry.addData("PID error", PID.getError(orientation.getYaw(AngleUnit.DEGREES)));
        robot.telemetry.update();
    }
}