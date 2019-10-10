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

/**

 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")

public class TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot           = new RobotHardware();   // Use a Pushbot's hardware


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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            sideways  =  gamepad1.left_stick_x;
            turn  =  gamepad1.right_stick_x;
            if (!gamepad1.a)
            {
                drive = drive*robot.NOTTURBOFACTOR;
                sideways = sideways*robot.NOTTURBOFACTOR;
                turn = turn*robot.NOTTURBOFACTOR;
            }

            if (Math.abs(drive) > robot.TELEOPDEADZONE)
            {
                robot.leftFrontDrive.setPower(Range.clip(drive, -1.0, 1.0));
                robot.rightFrontDrive.setPower(Range.clip(drive, -1.0, 1.0));
                robot.leftRearDrive.setPower(Range.clip(drive, -1.0, 1.0));
                robot.rightRearDrive.setPower(Range.clip(drive, -1.0, 1.0));
            }
            else if (Math.abs(sideways) > robot.TELEOPDEADZONE)
            {
                robot.leftFrontDrive.setPower(Range.clip(sideways, -1.0, 1.0));
                robot.rightFrontDrive.setPower(Range.clip(-sideways, -1.0, 1.0));
                robot.leftRearDrive.setPower(Range.clip(-sideways, -1.0, 1.0));
                robot.rightRearDrive.setPower(Range.clip(sideways, -1.0, 1.0));
            }
            else if (Math.abs(turn) > robot.TELEOPDEADZONE)
            {
                robot.leftFrontDrive.setPower(Range.clip(turn, -1.0, 1.0));
                robot.rightFrontDrive.setPower(Range.clip(-turn, -1.0, 1.0));
                robot.leftRearDrive.setPower(Range.clip(turn, -1.0, 1.0));
                robot.rightRearDrive.setPower(Range.clip(-turn, -1.0, 1.0));
            }
            else
            {
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.leftRearDrive.setPower(0);
                robot.rightRearDrive.setPower(0);
            }
            // Send telemetry message to signify robot running;
            telemetry.addData("leftFrontDrive",  "Offset = %.2f", robot.leftFrontDrive.getPower());
            telemetry.addData("rightFrontDrive",  "%.2f", robot.rightFrontDrive.getPower());
            telemetry.addData("leftRearDrive", "%.2f", robot.leftRearDrive.getPower());
            telemetry.addData("rightRearDrive", "%.2f", robot.rightRearDrive.getPower());

            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
