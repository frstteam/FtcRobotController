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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

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

@TeleOp(name="Practice - Auto Op", group = "Iterative Opmode")

public class PracticeAutoOpMode extends AbstractOpMode
{
    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
        double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise
        boolean movingToTarget  = false;
        boolean movedToTarget   = false;

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
            int targetTag = -1;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    //  Change from default DESIRED_TAG_ID tracking to targetTag tracking
                    //  if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    if (targetTag < 0) {
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
                // Determine heading and range error so we can use them to control the robot automatically.
                double  rangeError   = desiredTag.ftcPose.range;
                telemetry.addData("Range Error", rangeError);
                double  headingError = desiredTag.ftcPose.bearing - bearingCorrection;
                telemetry.addData("Bearing Error", headingError);

                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                drive = Range.clip(rangeError * AUTO_SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn  = Range.clip(headingError * AUTO_TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
                // drive using manual POV Joystick mode.
                // Flip left stick y and right stick x values to match Auto mode X and Yaw
                drive = MANUAL_DRIVE_SPEED * gamepad1.left_stick_y;  // Change drive rate by factor MANUAL_DRIVE_SPEED.
                turn  = MANUAL_TURN_SPEED * gamepad1.right_stick_x;  // Change turn rate by factor MANUAL_TURN_SPEED.
                telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drive, turn);
            }

            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            movingToTarget = true;
            moveRobot(drive, turn);
            sleep(10);

            if (gamepad2.a) {
                moveArmToHomePosition();
            }
            else if (gamepad2.b) {
                moveArmToIntakePosition();
            }
            else if (gamepad2.y) {
                moveArmToScoringPosition();
            }
            if (armLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                    armLeft.getTargetPosition() <= armShutdownThreshold &&
                    armLeft.getCurrentPosition() <= armShutdownThreshold
            ) {
                armLeft.setPower(0.0);
                armRight.setPower(0.0);
                armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

//            if (!leftDrive.isBusy() && !rightDrive.isBusy() && movingToTarget)
//                movingToTarget = false;
//
//            if (!leftDrive.isBusy() && !rightDrive.isBusy()
//                && targetFound && !movingToTarget  && !movedToTarget && !isStopRequested()) {
//                movedToTarget = true;
//                telemetry.addData("Status", "Scoring started");
//                moveArmToScoringPosition();
//                updateStatus("Moved arm. Sleeping 2 s");
//                sleep(2000);
//                gripper.setPosition(gripperOpenPosition);
//                updateStatus("Dropped pixel. Sleeping 2 s");
//                sleep(2000);
//                gripper.setPosition(gripperClosedPosition);
//                moveArmToHomePosition();
//                updateStatus("Scored. Sleeping 2 s");
//                sleep(2000);
//                //Watchdog to shut down motor once the arm reaches the home position
//                if (armLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
//                        armLeft.getTargetPosition() <= armShutdownThreshold &&
//                        armLeft.getCurrentPosition() <= armShutdownThreshold
//                ) {
//                    armLeft.setPower(0.0);
//                    armRight.setPower(0.0);
//                    armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                }
//                stop();
//                // TODO: Move robot to wing
//            }
        }
    }
}
