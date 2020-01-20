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

    public void down () throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_START);
        Thread.sleep (500);
    }
    public void up () throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_TOP);
        Thread.sleep(500);
    }




}
