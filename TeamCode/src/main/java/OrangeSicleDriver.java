
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="OrangeSicle")
public class OrangeSicleDriver extends LinearOpMode {
    RobotHardware robot           = new RobotHardware();   // Use a Pushbot's hardware

    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive())
            drive.forward(0.2,24);
        if (opModeIsActive())
            strafe.right(0.2, 24);
        if (opModeIsActive())
            drive.backward(0.2,24);
        if (opModeIsActive())
            strafe.left(0.2, 24);

    }
}
