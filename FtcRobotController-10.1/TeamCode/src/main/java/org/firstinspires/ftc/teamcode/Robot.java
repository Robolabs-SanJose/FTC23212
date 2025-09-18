package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "BetterMovement", group = "Linear OpMOde")
public class Robot extends LinearOpMode{
    private Drivebase drivebase = Drivebase.getInstance();

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    @Override
    public void runOpMode() {
        drivebase.initialize(this,
                "lf", "lb",
                "rf", "rb");

        while (opModeIsActive()) {
            drivebase.update(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y);
            drivebase.updateTelemetry(imuAngles, imu.getRobotAngularVelocity(AngleUnit.DEGREES), turnController);
        }
        Drivebase.reset();
    }
}