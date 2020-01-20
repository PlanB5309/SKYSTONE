import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;




public class BlockGrabber {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public BlockGrabber (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void grab () throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_GRAB);
        Thread.sleep (500);
    }
    public void release () throws InterruptedException {
        if (!linearOpMode.opModeIsActive())
            return;
        robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);
        Thread.sleep(500);
    }




}
