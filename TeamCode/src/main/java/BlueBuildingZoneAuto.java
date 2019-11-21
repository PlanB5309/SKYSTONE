import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "BlueBuildingZoneAuto", group = "Blue Auto")
public class BlueBuildingZoneAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    FoundationClaws foundationClaws =  new FoundationClaws(robot, telemetry, this);
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        drive.backward(0.2, 27);
        gyroTurn.absolute(0);
        drive.backward(0.2, 4);
        foundationClaws.down();
        drive.forward(0.2, 33);
        foundationClaws.up();
    }
}