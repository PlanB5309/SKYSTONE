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

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")

public class TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware

    enum Block_Mover {
        LIFT,
        ROTATE,
        LOWER,
        NOT_RUNNING;
    }

    @Override
    public void runOpMode() {
        double drive;
        double turn;
        double sideways;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        Block_Mover current_state = Block_Mover.NOT_RUNNING;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            sideways = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            if (!gamepad1.a) {
                drive = drive * robot.NOTTURBOFACTOR;
                sideways = sideways * robot.NOTTURBOFACTOR;
                turn = turn * robot.NOTTURBOFACTOR;
            }

            if (Math.abs(drive) > robot.TELEOPDEADZONE) {
                robot.leftFrontDrive.setPower(Range.clip(drive, -1.0, 1.0));
                robot.rightFrontDrive.setPower(Range.clip(drive, -1.0, 1.0));
                robot.leftRearDrive.setPower(Range.clip(drive, -1.0, 1.0));
                robot.rightRearDrive.setPower(Range.clip(drive, -1.0, 1.0));
            } else if (Math.abs(sideways) > robot.TELEOPDEADZONE) {
                robot.leftFrontDrive.setPower(Range.clip(sideways, -1.0, 1.0));
                robot.rightFrontDrive.setPower(Range.clip(-sideways, -1.0, 1.0));
                robot.leftRearDrive.setPower(Range.clip(-sideways, -1.0, 1.0));
                robot.rightRearDrive.setPower(Range.clip(sideways, -1.0, 1.0));
            } else if (Math.abs(turn) > robot.TELEOPDEADZONE) {
                robot.leftFrontDrive.setPower(Range.clip(turn, -1.0, 1.0));
                robot.rightFrontDrive.setPower(Range.clip(-turn, -1.0, 1.0));
                robot.leftRearDrive.setPower(Range.clip(turn, -1.0, 1.0));
                robot.rightRearDrive.setPower(Range.clip(-turn, -1.0, 1.0));
            } else {
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.leftRearDrive.setPower(0);
                robot.rightRearDrive.setPower(0);
            }
            // Send telemetry message to signify robot running;
            telemetry.addData("leftFrontDrive", "Offset = %.2f", robot.leftFrontDrive.getPower());
            telemetry.addData("rightFrontDrive", "%.2f", robot.rightFrontDrive.getPower());
            telemetry.addData("leftRearDrive", "%.2f", robot.leftRearDrive.getPower());
            telemetry.addData("rightRearDrive", "%.2f", robot.rightRearDrive.getPower());

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);

            if (Math.abs(gamepad2.right_stick_y) > robot.TELEOPDEADZONE) { //Move the lift up and down
                robot.liftMotor.setPower(Range.clip(gamepad2.right_stick_y, -1.0, 1.0));
            } else {
                robot.liftMotor.setPower(0);
            }

            if (gamepad2.a && !gamepad1.start) { //Grab a stone
                robot.skyStoneClaw.setPosition(robot.SKYSTONE_SERVO_DOWN);
            }
            if (gamepad2.y) { //Let go of the stone
                robot.skyStoneClaw.setPosition(robot.SKYSTONE_SERVO_UP);
            }

            if (gamepad2.b && !gamepad2.start) { //Turn on the wheels in the block intake
                robot.leftIntakeMotor.setPower(robot.INTAKE_WHEEL_SPEED);
                robot.rightIntakeMotor.setPower(robot.INTAKE_WHEEL_SPEED);
            } else {
                robot.leftIntakeMotor.setPower(0);
                robot.rightIntakeMotor.setPower(0);
            }


            if (gamepad2.x) {
                robot.blockKickerServo.setPosition(robot.KICKER_IN_POSITION);
            } else
                robot.blockKickerServo.setPosition(robot.KICKER_OUT_POSITION);


            if (gamepad2.left_bumper) { //Let go of the block
                robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);
            } else if (gamepad2.left_trigger > 0.5) { //Grab a block
                robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_GRAB);
            }

            if (gamepad2.dpad_up) { //Raise the block
                robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_TOP);
            } else if (gamepad2.dpad_right) { //Rotate the block to the outside (to place it on the foundation)
                robot.blockTurningServo.setPosition(robot.BLOCK_TURNING_SERVO_OUT);
            } else if (gamepad2.dpad_left) { //Rotate the block grabber to the inside
                robot.blockTurningServo.setPosition(robot.BLOCK_TURNING_SERVO_IN);
            } else if (gamepad2.dpad_down) { //Lower the block
                robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_START);
            }

            if (gamepad2.right_bumper) { //Swing out the capstone
                robot.capStoneServo.setPosition(robot.CAPSTONE_SERVO_OUT);
            } else if (gamepad2.right_trigger > 0.5) { //Swing in the capstone holder
                robot.capStoneServo.setPosition(robot.CAPSTONE_SERVO_IN);
            }
            if (gamepad2.left_stick_button)
            current_state = Block_Prepper(current_state);
            telemetry.addData("current_state", current_state);
            telemetry.update();
        }
    }


    private Block_Mover Block_Prepper(Block_Mover current_state) {
        switch (current_state) {
            case NOT_RUNNING:
                robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_TOP);
                return Block_Mover.LIFT;
            case LIFT:
                if (robot.blockFlippingServo.getPosition() == robot.LIFT_BLOCK_SERVO_TOP) {
                    robot.blockTurningServo.setPosition(robot.BLOCK_TURNING_SERVO_OUT);
                    return Block_Mover.ROTATE;
                }
            case ROTATE:
                if (robot.blockTurningServo.getPosition() == robot.BLOCK_TURNING_SERVO_OUT) {
                    robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_START);
                    return Block_Mover.LOWER;
                }
            case LOWER:
                if (robot.blockFlippingServo.getPosition() == robot.LIFT_BLOCK_SERVO_START)
                    return Block_Mover.NOT_RUNNING;
        }


        return current_state;
    }
}