
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorLineDrive {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public ColorLineDrive (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void drive (Double speed, int color) {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setPower(speed);
        robot.leftRearDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightRearDrive.setPower(speed);

        if (color == robot.COLOR_RED) {
            while (robot.colorSensor.red() < robot.COLOR_SENSOR_THRESHOLD && linearOpMode.opModeIsActive()) {
                Thread.yield();
                telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
                telemetry.update();
            }
        }
        else if (color == robot.COLOR_BLUE) {
            while (robot.colorSensor.blue() < robot.COLOR_SENSOR_THRESHOLD && linearOpMode.opModeIsActive()) {
                Thread.yield();
                telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
                telemetry.update();
            }
        }

        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
    }

}
