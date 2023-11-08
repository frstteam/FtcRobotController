/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a basic two-wheel Split Arcade Robot Drivetrain
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The driving goal is to rotate to keep the tag centered in the camera, while driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best exposure and gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named leftDrive and rightDrive.
 * The motor directions must be set so a positive power goes forward on both wheels;
 * This sample assumes that the default AprilTag Library (usually for the current season) is being loaded by default
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back, and the right stick will rotate the robot.
 * This is called POV Joystick mode, different than Tank Drive (where each joystick controls a wheel).
 *
 * Manually drive the robot until it displays Target data on the Driver Station.
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 *  Under "Drive To Target" mode, the robot has two goals:
 *  1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 *  2) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 *  Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the AUTO_SPEED_GAIN and AUTO_TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp(name="Auto Op", group = "Iterative Opmode")

public class AutoOpMode extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 9.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    private static final double AUTO_SPEED_GAIN =   0.5;   //  Speed Control "Gain". eg: 0.04 = Ramp up to 50% power at a 12.5 inch error.   (0.50 / 25.0)
    private static final double AUTO_TURN_GAIN  =   0.2;   //  Turn Control "Gain".  eg: 0.01 = Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    private static final double AUTO_ARM_GAIN = 0.8; // Arm movement "Gain"

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armLeft = null;
    private DcMotor armRight = null;
    private Servo gripper = null;
    private Servo wrist = null;

    private double armSetpoint = 0.0;

    private final double armManualDeadband = 0.03;

    private final double gripperClosedPosition = 1.0;
    private final double gripperOpenPosition = 0.5;
    private final double wristUpPosition = 1.0;
    private final double wristDownPosition = 0.0;

    private final int armHomePosition = 0;
    private final int armIntakePosition = 10;
    private final int armScorePosition = 520;
    private final int armShutdownThreshold = 5;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean rangeWhenTagFound = false;               // Used to determine range from the starting point, when the desired tag is found
    double bearingCorrection = 0;                    // Used to determine additional turn angle to center the robot in front of the tag
    private static final double MANUAL_DRIVE_SPEED = 0.5;
    private static final double MANUAL_TURN_SPEED = 0.5;

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // eg: Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    private static final int TAG_IMAGE_DECIMATION_10_FT = 1;
    private static final int TAG_IMAGE_DECIMATION_6_FT = 2;
    private static final int TAG_IMAGE_DECIMATION_4_FT = 3;
    private static final String DEVICE_LEFT_DRIVE = "leftDrive";
    private static final String DEVICE_RIGHT_DRIVE = "rightDrive";
    private static final String DEVICE_CAMERA_1 = "Webcam 1";
    private static final int TAG_BLUE_LEFT = 1;
    private static final int TAG_BLUE_CENTER = 2;
    private static final int TAG_BLUE_RIGHT = 3;
    private static final int TAG_RED_LEFT = 4;
    private static final int TAG_RED_CENTER = 5;
    private static final int TAG_RED_RIGHT = 6;
    // Correct target location to compensate the difference between
    // the Camera location and robot center
    private static final double X_CORRECTION = 0;

    private static final double COUNTS_PER_MOTOR_REV = 28.0;
    private static final double DRIVE_GEAR_REDUCTION = 30.21;
    private static final double WHEEL_CIRCUMFERENCE_MM = 90.0 * Math.PI;
    private static final double ROBOT_TRACK_WIDTH_MM = 300.0; // distance between the wheels
    private static final double DEGREES_TO_TURN = 180;

    private static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    private static final double COUNTS_PER_MM = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;
    private static final double TURN_DISTANCE = ROBOT_TRACK_WIDTH_MM * Math.PI * (DEGREES_TO_TURN / 360.0);
    private static final int TURN_COUNTS = (int)(TURN_DISTANCE * COUNTS_PER_MM / 2); // Number of counts to turn DEGREES_TO_TURN degrees

    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
        double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise

        initDevices();

        // Wait for the driver to press Start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            // Initialize targetTag to unknown
            // int targetTag = -1;
            // TODO: Reset targetTag to -1 after testing
            int targetTag = TAG_RED_LEFT;

            if (gamepad2.x) {
                // Go to Tag #4 - Red Left
                targetTag = TAG_RED_LEFT;
                telemetry.addData("Target Tag", TAG_RED_LEFT+ "(Red Left)");
                telemetry.update();
            }
            else if (gamepad2.y) {
                // Go to Tag #4 - Red Center
                targetTag = TAG_RED_CENTER;
                telemetry.addData("Target Tag", TAG_RED_CENTER + " (Red Center)");
                telemetry.update();
            }
            else if (gamepad2.b) {
                // Go to Tag #4 - Red Right
                targetTag = TAG_RED_RIGHT;
                telemetry.addData("Target Tag", TAG_RED_RIGHT + " (Red Right)");
                telemetry.update();
            }
            // Remove after testing
            else if (gamepad2.a) {
                // Go to Tag #2 - Blue Center
                targetTag = TAG_BLUE_CENTER;
                telemetry.addData("Target Tag", TAG_BLUE_CENTER + " (Blue Center)");
                telemetry.update();
            }
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    //  Change from default DESIRED_TAG_ID tracking to targetTag tracking
                    //  if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    if (detection.id == targetTag) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;

                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading and range error so we can use them to control the robot automatically.
                double  rangeError   = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                // Calculate bearing correction (as the camera is to the right of robot center)
                // Do not re-calculate it in every iteration
                if (!rangeWhenTagFound && X_CORRECTION != 0) {
                    telemetry.addData("Start Range", rangeError);
                    bearingCorrection = getBearingCorrection(X_CORRECTION, rangeError);
                    telemetry.addData("Bearing Correction", bearingCorrection);
                    rangeWhenTagFound = true;
                }
                telemetry.addData("Range Error", rangeError);
                double  headingError = desiredTag.ftcPose.bearing - bearingCorrection;
                telemetry.addData("Bearing Error", headingError);

                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                drive = Range.clip(rangeError * AUTO_SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn  = Range.clip(headingError * AUTO_TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
            } else {
                // drive using manual POV Joystick mode.
                // Flip left stick y and right stick x values to match Auto mode X and Yaw
                drive = MANUAL_DRIVE_SPEED * gamepad1.left_stick_y;  // Change drive rate by factor MANUAL_DRIVE_SPEED.
                turn  = MANUAL_TURN_SPEED * gamepad1.right_stick_x;  // Change turn rate by factor MANUAL_TURN_SPEED.
                telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drive, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, turn);
            sleep(10);
        }
        if (!leftDrive.isBusy() && !rightDrive.isBusy()) {
            telemetry.addData("Status", "Scoring started");
            moveArmToScorePosition();
            gripper.setPosition(gripperOpenPosition);
            sleep(10);
            gripper.setPosition(gripperClosedPosition);
            moveArmToHomePosition();
            telemetry.addData("Status", "Scoring completed");
        }
    }

    /**
     * Find bearing angle correction given X correction and range
     */
    private double getBearingCorrection(double xCorrection, double range) {
        double angleRadians = Math.asin(xCorrection / range);
        // Turn 180 degrees as the camera is on the front, but
        // the scoring position needs the back facing the board
        double angleDegrees = Math.toDegrees(angleRadians);

        return angleDegrees;
    }
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x + yaw;
        double rightPower   = x - yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Send powers to the wheels.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    private void moveArmToIntakePosition() {
        telemetry.addData("Status", "Arm intake starting");
        armLeft.setTargetPosition(armIntakePosition);
        armRight.setTargetPosition(armIntakePosition);
        armLeft.setPower(AUTO_ARM_GAIN);
        armRight.setPower(AUTO_ARM_GAIN);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(wristDownPosition);
        telemetry.addData("Status", "Arm intake done");
    }

    private void moveArmToScorePosition() {
        telemetry.addData("Status", "Scoring");
        armLeft.setTargetPosition(armScorePosition);
        armRight.setTargetPosition(armScorePosition);
        armLeft.setPower(AUTO_ARM_GAIN);
        armRight.setPower(AUTO_ARM_GAIN);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(wristUpPosition);
        telemetry.addData("Status", "Scored");
    }

    private void moveArmToHomePosition() {
        telemetry.addData("Status", "Arm moving to home");
        armLeft.setTargetPosition(armHomePosition);
        armRight.setTargetPosition(armHomePosition);
        armLeft.setPower(AUTO_ARM_GAIN);
        armRight.setPower(AUTO_ARM_GAIN);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(wristUpPosition);
        telemetry.addData("Status", "Arm at home");
    }

    private void initDevices() {
        telemetry.addData("Status", "Initializing");

        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        armLeft  = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        gripper = hardwareMap.get(Servo.class, "gripper");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // To drive forward, most robots need the motor on one side to be reversed because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Single Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armLeft.setDirection(DcMotor.Direction.FORWARD);
        armRight.setDirection(DcMotor.Direction.REVERSE);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLeft.setPower(0.0);
        armRight.setPower(0.0);

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Don't use encoders for both motors initially
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        telemetry.addData("Status", "Initialized");
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(TAG_IMAGE_DECIMATION_10_FT);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, DEVICE_CAMERA_1))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }
}
