import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StopOnLine {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public StopOnLine (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void forward(double speed, int distance) throws InterruptedException{
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

        int red = robot.colorSensor.red();
        int blue  = robot.colorSensor.blue();

        telemetry.addData("linear opmode is working, target = ", target);

//       Thread.sleep(5000);
//        telemetry.addData("", target);
        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() && red < robot.RED_THRESHOLD && blue < robot.BLUE_THRESHOLD)  {

            red = robot.colorSensor.red();
            blue  = robot.colorSensor.blue();
            Thread.yield();
            telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("blue value", blue);
            telemetry.addData("red value", red);
            telemetry.update();
        }

        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);

    }

    public void backward(double speed, int distance) {
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

        int red = robot.colorSensor.red();
        int blue  = robot.colorSensor.blue();

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

//       Thread.sleep(5000);
//        telemetry.addData("", target);
        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() && red < robot.RED_THRESHOLD && blue < robot.BLUE_THRESHOLD)  {

            blue  = robot.colorSensor.blue();
            red = robot.colorSensor.red();
            Thread.yield();
            telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("blue value", blue);
            telemetry.addData("red value", red);
            telemetry.update();
        }

        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);

    }

    public void strafeRight(double speed, int distance) {
        setupDriveTrain();

        int target = distance * robot.STRAFE_CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(target);
        robot.leftRearDrive.setTargetPosition(-target);
        robot.rightFrontDrive.setTargetPosition(-target);
        robot.rightRearDrive.setTargetPosition(target);

        robot.leftFrontDrive.setPower(speed);
        robot.leftRearDrive.setPower(-speed);
        robot.rightFrontDrive.setPower(-speed);
        robot.rightRearDrive.setPower(speed);

        int red = robot.colorSensor.red();
        int blue  = robot.colorSensor.blue();

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

//       Thread.sleep(5000);
//        telemetry.addData("", target);
        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() && red < robot.RED_THRESHOLD && blue < robot.BLUE_THRESHOLD)  {

            blue  = robot.colorSensor.blue();
            red = robot.colorSensor.red();
            Thread.yield();
            telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("blue value", blue);
            telemetry.addData("red value", red);
            telemetry.update();
        }

        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);

    }

    public void strafeLeft(double speed, int distance) {
        setupDriveTrain();

        int target = distance * robot.STRAFE_CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(-target);
        robot.leftRearDrive.setTargetPosition(target);
        robot.rightFrontDrive.setTargetPosition(target);
        robot.rightRearDrive.setTargetPosition(-target);

        robot.leftFrontDrive.setPower(-speed);
        robot.leftRearDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightRearDrive.setPower(-speed);

        int red = robot.colorSensor.red();
        int blue  = robot.colorSensor.blue();

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

//       Thread.sleep(5000);
//        telemetry.addData("", target);
        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() && red < robot.RED_THRESHOLD && blue < robot.BLUE_THRESHOLD)  {

            blue  = robot.colorSensor.blue();
            red = robot.colorSensor.red();
            Thread.yield();
            telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("blue value", blue);
            telemetry.addData("red value", red);
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
