
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="OrangeSicleDriver")
public class OrangeSicleDriver extends LinearOpMode {
    RobotHardware robot           = new RobotHardware();   // Use a Pushbot's hardware
    LinearOpMode linearOpMode= new LinearOpMode() {
        @Override
        public void runOpMode() throws InterruptedException {

        }
    };

    Drive drive = new Drive(robot, telemetry, linearOpMode);
    Strafe strafe = new Strafe(robot, telemetry, linearOpMode);
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        waitForStart();
        drive.forward(0.2,36);
        strafe.right(0.2, 36);
        drive.backward(0.2,36);
        strafe.left(0.2, 36);
    }
}
