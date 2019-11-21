import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FoundationClaws {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public FoundationClaws (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }
    public void down () throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_DOWN);
        robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_DOWN);
        Thread.sleep (500);
    }
    public void up () throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_UP);
        robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_UP);
        Thread.sleep(500);
    }

}


