import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class StopAtDistance {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public StopAtDistance (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void strafe(double speed, int sensorDistance, int maxDistance) throws InterruptedException{
        setupDriveTrain();

        int target = maxDistance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(target);
        robot.leftRearDrive.setTargetPosition(target);
        robot.rightFrontDrive.setTargetPosition(target);
        robot.rightRearDrive.setTargetPosition(target);

        robot.leftFrontDrive.setPower(speed);
        robot.leftRearDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightRearDrive.setPower(speed);


        telemetry.addData("linear opmode is working, target = ", target);
        double distanceValue = robot.sideDistanceSensor.getDistance(DistanceUnit.CM);

//       Thread.sleep(5000);
//        telemetry.addData("", target);
        telemetry.addData("Distance (cm) ",
                String.format(Locale.US, "%.02f", robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));

        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() && sensorDistance < distanceValue)  {
            distanceValue = robot.sideDistanceSensor.getDistance(DistanceUnit.CM);
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
