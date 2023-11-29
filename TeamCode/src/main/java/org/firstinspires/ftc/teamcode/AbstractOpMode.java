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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import java.util.ArrayList;
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

@TeleOp(name="Abstract Generic Op", group = "Iterative Opmode")
@Disabled
public abstract class AbstractOpMode extends LinearOpMode
{
    public enum Alliance {
        RED,
        BLUE
    }
    protected Alliance alliance;
    protected int minTargetTagId;  // Min id of acceptable target tag
    protected int maxTargetTagId;  // Max id of acceptable target tag

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 2.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    static final double AUTO_SPEED_GAIN =   0.02;   //  Speed Control "Gain". eg: 0.04 = Ramp up to 50% power at a 12.5 inch error.   (0.50 / 25.0)
    static final double AUTO_TURN_GAIN  =   0.01;   //  Turn Control "Gain".  eg: 0.01 = Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    static final double AUTO_ARM_GAIN = 0.7; // Arm movement "Gain"

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor leftDrive = null;
    protected DcMotor rightDrive = null;
    protected DcMotor armLeft = null;
    protected DcMotor armRight = null;
    protected Servo gripper = null;
    protected Servo wrist = null;

    protected double armSetpoint = 0.0;

    protected final double armManualDeadband = 0.03;

    protected final double gripperClosedPosition = 1.0;
    protected final double gripperOpenPosition = 0.5;
    protected final double wristUpPosition = 1.0;
    protected final double wristDownPosition = 0.0;

    protected final int armHomePosition = 0;
    protected final int armIntakePosition = 10;
    protected final int armScorePosition = 520;
    protected final int armShutdownThreshold = 5;

    static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    protected VisionPortal visionPortal;               // Used to manage the video source.
    protected AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    protected AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean rangeWhenTagFound = false;               // Used to determine range from the starting point, when the desired tag is found
    double bearingCorrection = 0;                    // Used to determine additional turn angle to center the robot in front of the tag
    static final double MANUAL_DRIVE_SPEED = 0.5;
    static final double MANUAL_TURN_SPEED = 0.5;

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // eg: Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    static final int TAG_IMAGE_DECIMATION_10_FT = 1;
    static final int TAG_IMAGE_DECIMATION_6_FT = 2;
    static final int TAG_IMAGE_DECIMATION_4_FT = 3;
    static final String DEVICE_LEFT_DRIVE = "leftDrive";
    static final String DEVICE_RIGHT_DRIVE = "rightDrive";
    static final String DEVICE_CAMERA_1 = "Webcam 1";
    static final int TAG_BLUE_LEFT = 1;
    static final int TAG_BLUE_CENTER = 2;
    static final int TAG_BLUE_RIGHT = 3;
    static final int TAG_RED_LEFT = 4;
    static final int TAG_RED_CENTER = 5;
    static final int TAG_RED_RIGHT = 6;
    // Correct target location to compensate the difference between
    // the Camera location and robot center
    static final double X_CORRECTION = 0;

    static final double COUNTS_PER_MOTOR_REV = 28.0;
    static final double DRIVE_GEAR_REDUCTION = 30.21;
    static final double WHEEL_CIRCUMFERENCE_MM = 90.0 * Math.PI;
    static final double ROBOT_TRACK_WIDTH_MM = 300.0; // distance between the wheels
    static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double COUNTS_PER_MM = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;

    static final int STOP_INTERVAL = 50; // Number of iterations for which range error should be constant, to determine that the robot has stopped.
    static final double RANGE_ERROR_TOLERANCE = 0.001; // Range error will be considered constant if it's within this tolerance

    protected double parkTurnDegrees; // Turn this much to park after scoring
    protected double parkTurnMm; // Turn this much to park after scoring - value for leftDrive
    protected double parkTurnTimeoutSec;
    protected double parkDriveMm; // Drive this much to park after scoring
    protected double parkDriveTimeoutSec;

    /**
     * Initialize devices
     */
    protected void initDevices() {
        updateStatus("Initializing");

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

        updateStatus("Initialized");
    }   /**
     * Do this after initializing devices, before start button is tapped
     */

    protected void waitBeforeStart() {
        // Wait for the driver to press Start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
    }

    /**
     * Detect target tag and move toward it
     */
    protected void moveToTargetTag() {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
        double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise
        boolean movedToTarget   = false;

        int iteration = 0;
        runtime.reset();
        ArrayList<Double> rangeErrors = new ArrayList<Double>(); // Used to find if rangeError is constant across iterations, i.e. if robot has stopped.

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;
            // Initialize targetTag to unknown
            // int targetTag = -1;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id >= minTargetTagId && detection.id <= maxTargetTagId) {
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
            telemetry.addData("Iteration:Time", "%d:%.2f", ++iteration, runtime.time());

            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                // Determine heading and range error so we can use them to control the robot automatically.
                double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                // Remember current range error for every 10 iterations
                if (iteration % 10 == 0) {
                    rangeErrors.add(rangeError);
                }
                telemetry.addData("Range Error", rangeError);
                double headingError = desiredTag.ftcPose.bearing - bearingCorrection;
                telemetry.addData("Bearing Error", headingError);

                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                drive = Range.clip(rangeError * AUTO_SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * AUTO_TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            }
//            else if (!movedToTarget) {
//                // Turn to try to find a acceptable target tag
//                turn = Range.clip(0.2, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//                sleep(20);
//            }

            telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, turn);
            sleep(10);

            // Check if the range error is constant for a few iterations, before trying to score
            int numRangeErrors = rangeErrors.size();
            if (numRangeErrors > STOP_INTERVAL
                    && rangeErrors.get(numRangeErrors - 1)
                    - rangeErrors.get(numRangeErrors - (STOP_INTERVAL + 1)) <= RANGE_ERROR_TOLERANCE) {
                updateStatus("Constant range error");
                stopRobot();
                movedToTarget = true;
                break;
            }
        }
    }
    /**
     * Find bearing angle correction given X correction and range
     */
    protected double getBearingCorrection(double xCorrection, double range) {
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

    protected void moveArmToIntakePosition() {
        updateStatus("Arm intake starting");
        armLeft.setTargetPosition(armIntakePosition);
        armRight.setTargetPosition(armIntakePosition);
        armLeft.setPower(AUTO_ARM_GAIN);
        armRight.setPower(AUTO_ARM_GAIN);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(wristDownPosition);
        updateStatus("Arm intake done");
    }

    protected void moveArmToScoringPosition() {
        updateStatus("Scoring");
        armLeft.setTargetPosition(armScorePosition);
        armRight.setTargetPosition(armScorePosition);
        armLeft.setPower(AUTO_ARM_GAIN);
        armRight.setPower(AUTO_ARM_GAIN);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(wristUpPosition);
        updateStatus("Scored");
    }

    protected void moveArmToHomePosition() {
        updateStatus("Arm moving to home");
        armLeft.setTargetPosition(armHomePosition);
        armRight.setTargetPosition(armHomePosition);
        armLeft.setPower(AUTO_ARM_GAIN);
        armRight.setPower(AUTO_ARM_GAIN);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(wristUpPosition);
        updateStatus("Arm at home");
    }

    /**
     * Initialize the AprilTag processor.
     */
    protected void initAprilTag() {
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
    protected void setManualExposure(int exposureMS, int gain) {
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

    protected void updateStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }

    protected void stopRobot() {
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    protected void score() {
        // Scoring
        if (opModeIsActive()) {
            updateStatus("Scoring started");
            moveArmToScoringPosition();
            updateStatus("Moved arm. Sleeping 4 s");
            sleep(4000);
            gripper.setPosition(gripperOpenPosition);
            updateStatus("Dropped pixel. Sleeping 3 s");
            sleep(3000);
            gripper.setPosition(gripperClosedPosition);
            moveArmToHomePosition();
            updateStatus("Scored. Stopping in 3 s");
            sleep(3000);
            //Watchdog to shut down motor once the arm reaches the home position
            if (armLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                    armLeft.getTargetPosition() <= armShutdownThreshold &&
                    armLeft.getCurrentPosition() <= armShutdownThreshold
            ) {
                armLeft.setPower(0.0);
                armRight.setPower(0.0);
                armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            // stop();
        }
    }

    protected void setAllianceParams() {
        if (alliance != null) {
            switch (alliance) {
                case RED:
                    // TODO - Detect actual spike mark tag
                    minTargetTagId = TAG_RED_CENTER;
                    maxTargetTagId = TAG_RED_RIGHT;
                    parkTurnDegrees = 30;
                    parkTurnMm = 100;
                    parkDriveMm = -150;
                    parkTurnTimeoutSec = 3.0;
                    parkDriveTimeoutSec = 3.0;
                    break;
                case BLUE:
                    // TODO - Detect actual spike mark tag
                    minTargetTagId = TAG_BLUE_LEFT;
                    maxTargetTagId = TAG_BLUE_CENTER;
                    parkTurnDegrees = 30;
                    parkTurnMm = -100;
                    parkDriveMm = -150;
                    parkTurnTimeoutSec = 3.0;
                    parkDriveTimeoutSec = 3.0;
                    break;
                default:
                    updateStatus("Invalid Alliance");
            }
        }
        else {
            updateStatus("Invalid Alliance");
        }
    }

    /**
     * Calculate turn counts (ticks) given turn degrees
     * @param turnDegrees
     */
    protected int getTurnCounts(double turnDegrees) {
        double turnDistance = ROBOT_TRACK_WIDTH_MM * Math.PI * (turnDegrees / 360.0);
        int turnCounts = (int)(turnDistance * COUNTS_PER_MM / 2); // Number of counts to turn turnDegrees degrees
        return turnCounts;
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftMm, double rightMm,
                             double timeoutSec) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftMm * COUNTS_PER_MM);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightMm * COUNTS_PER_MM);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSec) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    protected void park() {
        updateStatus("Parking");
        // Park in an acceptable place

        // int turnCounts = getTurnCounts(parkTurnDegrees); // Number of counts to turn parkTurnDegrees degrees
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(MAX_AUTO_TURN,   parkTurnMm, -parkTurnMm, parkTurnTimeoutSec);  // S2: Turn Right 10 cm with 3 Sec timeout
        encoderDrive(MAX_AUTO_SPEED,  parkDriveMm,  parkDriveMm, parkDriveTimeoutSec);  // S1: Reverse 47 Inches with 3 Sec timeout
        updateStatus("Parked");
    }
}
