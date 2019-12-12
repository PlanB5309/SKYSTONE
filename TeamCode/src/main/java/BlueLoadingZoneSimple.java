import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Loading Zone Simple", group = "Blue Auto")
public class BlueLoadingZoneSimple extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        strafe.right(0.2, 2);
        gyroTurn.absolute(0);
        stopOnLine.forward(0.2, 48);

    }
}