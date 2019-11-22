
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class Drive {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public Drive (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void forward(double speed, int distance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;
       setupDriveTrain();

       int target = distance * robot.CLICKS_PER_INCH;
       robot.leftFrontDrive.setTargetPosition(target);
       robot.leftRearDrive.setTargetPosition(target);
       robot.rightFrontDrive.setTargetPosition(target);
       robot.rightRearDrive.setTargetPosition(target);

       robot.leftFrontDrive.setPower(speed);
       robot.leftRearDrive.setPower(speed);
       robot.rightFrontDrive.setPower(speed);
       robot.rightRearDrive.setPower(speed);

       telemetry.addData("linear opmode is working, target = ", target);

//       Thread.sleep(5000);
//        telemetry.addData("", target);
        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive()) {
           Thread.yield();
           telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
           telemetry.update();
       }

       robot.leftFrontDrive.setPower(0);
       robot.leftRearDrive.setPower(0);
       robot.rightFrontDrive.setPower(0);
       robot.rightRearDrive.setPower(0);

    }

    public void backward(double speed, int distance) {
        if (!linearOpMode.opModeIsActive())
            return;
        setupDriveTrain();

        int target = distance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(-target);
        robot.leftRearDrive.setTargetPosition(-target);
        robot.rightFrontDrive.setTargetPosition(-target);
        robot.rightRearDrive.setTargetPosition(-target);

        robot.leftFrontDrive.setPower(-speed);
        robot.leftRearDrive.setPower(-speed);
        robot.rightFrontDrive.setPower(-speed);
        robot.rightRearDrive.setPower(-speed);

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

//       Thread.sleep(5000);
//        telemetry.addData("", target);
        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive()) {
            Thread.yield();
            telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
            telemetry.update();
        }

        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);

    }
    private void setupDriveTrain () {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
