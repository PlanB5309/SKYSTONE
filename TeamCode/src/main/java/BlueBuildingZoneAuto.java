import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "BlueBuildingZoneAuto", group = "Red Auto")
public class BlueBuildingZoneAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        strafe.right(0.2, 27);
        gyroTurn.absolute(0);
        strafe.right(0.2, 4);
        skyStoneClaw.down();
        strafe.left(0.2, 31);
        skyStoneClaw.up();
        stopOnLine.backward(0.2, 48);
    }
}