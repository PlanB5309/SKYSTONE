import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Building Zone Autonomous")
public class RedBuildingZoneAuto extends LinearOpMode{//SkyStoneClaw.down
    RobotHardware robot           = new RobotHardware();   // Use a Pushbot's hardware
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw(robot, telemetry, this);

    public void runOpMode () throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive())
            strafe.left(0.5, 25);
        if (opModeIsActive())
            skyStoneClaw.down();
        if (opModeIsActive())
            strafe.right(0.5, 6);
        if (opModeIsActive())
            drive.forward(0.5, 48);
    }
}
