import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Blue Building Zone Simple", group = "1 Blue Auto")
public class BlueBuildingZoneSimple extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe (robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    FoundationClaws foundationClaws =  new FoundationClaws(robot, telemetry, this);
    StopAtDistance stopAtDistance = new StopAtDistance (robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        drive.forward(0.15,33);
        strafe.left(0.15, 12);
        foundationClaws.down();
        drive.backward(0.15, 40);
        foundationClaws.up();
        strafe.right(0.15,30);
        drive.forward(0.15, 23);
        strafe.left(0.3, 10);
        stopOnLine.strafeRight(0.15,48);
    }
}