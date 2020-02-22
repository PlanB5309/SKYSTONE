import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Red Building Zone Simple", group = "2 Red Auto")
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
        drive.forward(0.15,32);
        strafe.right(0.15, 12);
        foundationClaws.down();
        drive.backward(0.15, 40);
        foundationClaws.up();
        strafe.left(0.15,30);
        drive.forward(0.15, 23);
        strafe.right(0.3, 10);
        stopOnLine.strafeLeft(0.15,48);
    }
}