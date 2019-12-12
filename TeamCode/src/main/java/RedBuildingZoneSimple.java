import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "RedBuildingZoneSimple", group = "Red Auto")
public class RedBuildingZoneSimple extends LinearOpMode{
    RobotHardware robot           = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe (robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    FoundationClaws foundationClaws =  new FoundationClaws(robot, telemetry, this);
    public void runOpMode () throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        drive.backward(0.2, 27);
        gyroTurn.absolute(0);
        strafe.left(0.2, 12);
        gyroTurn.absolute(0);
        drive.backward(0.2, 4);
        foundationClaws.down();
        drive.forward(0.2, 33);
        foundationClaws.up();
        drive.backward (0.2, 1);
        gyroTurn.absolute(-4);

        stopOnLine.strafeRight(0.3, 48);
    }
}