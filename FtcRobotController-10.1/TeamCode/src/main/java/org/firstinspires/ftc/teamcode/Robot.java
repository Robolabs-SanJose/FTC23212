package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "BetterMovement", group = "Linear OpMOde")
public class Robot extends LinearOpMode{
    private Drivebase drivebase = Drivebase.getInstance();
    private PIDController turnController;
    IMU imu = hardwareMap.get(IMU.class,"imu");

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    @Override
    public void runOpMode() {
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        drivebase.initialize(this,
                "lf", "lb",
                "rf", "rb");
        turnController = new PIDController(0.01, 0, 0);// Tune Later

        while (opModeIsActive()) {
            YawPitchRollAngles imuAngles = imu.getRobotYawPitchRollAngles();
            double imuData = imuAngles.getYaw(AngleUnit.DEGREES);
            imuData = -imuData;
            if (imuData < 0) {
                imuData += 360;
            }

            double rightStickX = gamepad1.right_stick_x;
            double rightStickY = gamepad1.right_stick_y;
            if ((rightStickX != 0) && (rightStickY != 0)){
                double heading = atan2(rightStickX, -rightStickY);
                heading *= 180/PI;
                turnController.setSetpoint(heading);
            }

            double turning = turnController.update(imuData, true);
            drivebase.update(gamepad1.left_stick_y, -gamepad1.left_stick_x, turning);
            drivebase.updateTelemetry(imuAngles, imu.getRobotAngularVelocity(AngleUnit.DEGREES), turnController);
        }
        Drivebase.reset();
    }
}