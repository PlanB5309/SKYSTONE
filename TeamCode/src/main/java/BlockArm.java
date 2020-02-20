import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;




public class BlockArm {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public BlockArm (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void down() throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_DOWN);
        Thread.sleep (500);
    }
    public void up() throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
        Thread.sleep(500);
    }
    public void middle() throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_BARLEY_UP);
        Thread.sleep(250);
    }




}
