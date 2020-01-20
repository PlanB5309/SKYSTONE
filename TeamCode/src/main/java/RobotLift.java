
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RobotLift {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public RobotLift(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void up(int height) {

        if (!linearOpMode.opModeIsActive())
            return;

        robot.runUsingEncoder();
        robot.setupDriveTrain();

        int target = height * LIFT_CLICKS_PER_INCH;

        robot.liftMotor.setTargetPosition(target);

        while (robot.liftMotor.isBusy() && linearOpMode.opModeIsActive()) {
            Thread.yield();
        }
        robot.stop();


    }

    public void down() {

        if (!linearOpMode.opModeIsActive())
            return;

        robot.runUsingEncoder();
        robot.setupDriveTrain();

        int target = 0;

        robot.liftMotor.setTargetPosition(target);

        while (robot.liftMotor.isBusy() && linearOpMode.opModeIsActive()) {
            Thread.yield();
        }
        robot.stop();

    }
}
