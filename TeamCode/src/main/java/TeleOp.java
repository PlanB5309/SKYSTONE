/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")

public class TeleOp extends LinearOpMode {
    Long startTime;
    Long startTime2;

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware

    enum Block_Mover {
        LIFT,
        ROTATE,
        LOWER,
        NOT_RUNNING;
    }

    enum Reverse_Block_Mover {
        LIFT,
        ROTATE,
        LOWER,
        NOT_RUNNING;
    }

    enum Servo_State {
        IN,
        OUT;
    }

    @Override
    public void runOpMode() {
        double ly;
        double rx;
        double lx;
        boolean slow_mode;
        boolean normal_mode;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            slow_mode = false;
            normal_mode = false;
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

//            if (Math.abs(drive) > robot.TELEOPDEADZONE) {
//                robot.leftFrontDrive.setPower(Range.clip(drive, -1.0, 1.0));
//                robot.rightFrontDrive.setPower(Range.clip(drive, -1.0, 1.0));
//                robot.leftRearDrive.setPower(Range.clip(drive, -1.0, 1.0));
//                robot.rightRearDrive.setPower(Range.clip(drive, -1.0, 1.0));
//            } else if (Math.abs(sideways) > robot.TELEOPDEADZONE) {
//                robot.leftFrontDrive.setPower(Range.clip(sideways, -1.0, 1.0));
//                robot.rightFrontDrive.setPower(Range.clip(-sideways, -1.0, 1.0));
//                robot.leftRearDrive.setPower(Range.clip(-sideways, -1.0, 1.0));
//                robot.rightRearDrive.setPower(Range.clip(sideways, -1.0, 1.0));
//            } else if (Math.abs(turn) > robot.TELEOPDEADZONE) {
//                robot.leftFrontDrive.setPower(Range.clip(turn, -1.0, 1.0));
//                robot.rightFrontDrive.setPower(Range.clip(-turn, -1.0, 1.0));
//                robot.leftRearDrive.setPower(Range.clip(turn, -1.0, 1.0));
//                robot.rightRearDrive.setPower(Range.clip(-turn, -1.0, 1.0));

            //strafe and turn right slowly with dpad
            if (gamepad1.dpad_right ||
                    gamepad1.dpad_left ||
                    gamepad1.dpad_up ||
                    gamepad1.dpad_down) {
                slow_mode = true;
                normal_mode = false;


                if (gamepad1.dpad_right) {
                    //turn right slowly with dpad
                    if (gamepad1.b) {
                        robot.leftFrontDrive.setPower(0.05);
                        robot.rightFrontDrive.setPower(-0.05);
                        robot.leftRearDrive.setPower(0.05);
                        robot.rightRearDrive.setPower(-0.05);
                    }
                    //strafe right slowly with dpad
                    else {
                        robot.leftFrontDrive.setPower(0.05);
                        robot.rightFrontDrive.setPower(-0.05);
                        robot.leftRearDrive.setPower(-0.05);
                        robot.rightRearDrive.setPower(0.05);
                    }
                    //strafe and turn left slowly with dpad
                } else if (gamepad1.dpad_left) {
                    //turn left slowly with dpad
                    if (gamepad1.b) {
                        robot.leftFrontDrive.setPower(-0.05);
                        robot.rightFrontDrive.setPower(0.05);
                        robot.leftRearDrive.setPower(-0.05);
                        robot.rightRearDrive.setPower(0.05);
                    }
                    //strafe left slowly with dpad
                    else {
                        robot.leftFrontDrive.setPower(-0.05);
                        robot.rightFrontDrive.setPower(0.05);
                        robot.leftRearDrive.setPower(0.05);
                        robot.rightRearDrive.setPower(-0.05);
                    }
                    //drive forward slowly with dpad
                } else if (gamepad1.dpad_up) {
                    robot.leftFrontDrive.setPower(0.07);
                    robot.rightFrontDrive.setPower(0.07);
                    robot.leftRearDrive.setPower(0.07);
                    robot.rightRearDrive.setPower(0.07);
                    //drive backward slowly with dpad
                } else if (gamepad1.dpad_down) {
                    robot.leftFrontDrive.setPower(-0.07);
                    robot.rightFrontDrive.setPower(-0.07);
                    robot.leftRearDrive.setPower(-0.07);
                    robot.rightRearDrive.setPower(-0.07);
                }
            }
            // Normal mode
            else {
                ly = -gamepad1.left_stick_y; //drive forward
                lx = -gamepad1.left_stick_x; //strafe
                rx = -gamepad1.right_stick_x; //turn

                if (Math.abs(ly) > robot.TELEOPDEADZONE ||
                        Math.abs(lx) > robot.TELEOPDEADZONE ||
                        Math.abs(rx) > robot.TELEOPDEADZONE) {

                    normal_mode = true;
                    slow_mode = false;
                    // Compute the drive speed of each drive motor based on formula from redit
                    double FL_power_raw = ly - lx - (rx * .7f);
                    double FR_power_raw = ly + lx + (rx * .7f);
                    double RL_power_raw = ly + lx - (rx * .7f);
                    double RR_power_raw = ly - lx + (rx * .7f);

                    //Clip the values generated by the above formula so that they never go outisde of -1 to 1
                    double FL_power = Range.clip(FL_power_raw, -1, 1);
                    double FR_power = Range.clip(FR_power_raw, -1, 1);
                    double RL_power = Range.clip(RL_power_raw, -1, 1);
                    double RR_power = Range.clip(RR_power_raw, -1, 1);

                    if (!gamepad1.a) { //This is normal mode
                        FL_power = FL_power * robot.NOTTURBOFACTOR;
                        FR_power = FR_power * robot.NOTTURBOFACTOR;
                        RL_power = RL_power * robot.NOTTURBOFACTOR;
                        RR_power = RR_power * robot.NOTTURBOFACTOR;
                    }

                    robot.leftFrontDrive.setPower(FL_power);
                    robot.rightFrontDrive.setPower(FR_power);
                    robot.leftRearDrive.setPower(RL_power);
                    robot.rightRearDrive.setPower(RR_power);

                }
                if (slow_mode == false && normal_mode == false) {
                    robot.leftFrontDrive.setPower(0);
                    robot.rightFrontDrive.setPower(0);
                    robot.leftRearDrive.setPower(0);
                    robot.rightRearDrive.setPower(0);
                }
            }


            if (Math.abs(gamepad2.right_stick_y) > robot.TELEOPDEADZONE) { //Move the lift up and down
                robot.liftMotor.setPower(Range.clip(gamepad2.right_stick_y, -1.0, 1.0));
            } else {
                robot.liftMotor.setPower(0);
            }


            if (gamepad1.left_trigger > 0.5) { // Grab the foundation
                robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_DOWN);
                robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_DOWN);
            }

            if (gamepad1.left_bumper) { // Let go of the foundation
                robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_UP);
                robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_UP);
            }


            if (gamepad2.left_bumper) { //Let go of the block
                robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);

            } else if (gamepad2.left_trigger > 0.5) { //Grab a block
                robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_GRAB);
            }

            if (gamepad2.dpad_up) {
                robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
            }

            if (gamepad2.dpad_down) {
                robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_DOWN);
            }


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);

        }
    }
}