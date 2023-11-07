package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * First test op mode
 */
@TeleOp(name="FirstTest", group = "Concept")
@Disabled
public class FirstOpMode extends LinearOpMode {
    private DcMotor revCoreHexMotor;
    private RevColorSensorV3 revColorSensorV3;
    private RevTouchSensor revTouchSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        revCoreHexMotor = hardwareMap.get(DcMotor.class, "revCoreHexMotor");
        revColorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "revColorSensorV3");
        revTouchSensor = hardwareMap.get(RevTouchSensor.class, "revTouchSensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double targetPower = 0;
        double targetPosition = 0;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Color - Light", revColorSensorV3.getLightDetected());
            telemetry.addData("Color - Distance (mm)", revColorSensorV3.getDistance(DistanceUnit.MM));
            telemetry.addData("Touch - Value", revTouchSensor.getValue());
            telemetry.addData("Touch - Pressed", revTouchSensor.isPressed());

            telemetry.addLine();

            targetPower = -gamepad1.left_stick_y;
            targetPosition = gamepad1.right_stick_x;
            if (Math.abs(targetPower) > 0) {
                revCoreHexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                revCoreHexMotor.setPower(targetPower);
            }
            else if (Math.abs(targetPosition) > 0) {
                revCoreHexMotor.setPower(0.25);
                revCoreHexMotor.setTargetPosition(revCoreHexMotor.getCurrentPosition() + (int) targetPosition * 100);
                revCoreHexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("Target Power", targetPower);
            telemetry.addData("Motor Power", revCoreHexMotor.getPower());
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Motor Position", revCoreHexMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
