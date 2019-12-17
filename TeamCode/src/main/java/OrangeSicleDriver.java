
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name="OrangeSicle")
public class OrangeSicleDriver extends LinearOpMode {
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    StopAtDistance stopAtDistance = new StopAtDistance(robot, telemetry, this);
    BlockIntake blockIntake = new BlockIntake(robot, telemetry, this);
    BlockFlipper blockFlipper = new BlockFlipper(robot, telemetry, this);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        stopAtDistance.strafe(0.1, 5, 10);
        gyroTurn.absolute(0);
        blockIntake.startSucking();
        strafe.right(0.2, 12);
        drive.backward(0.2, 5);
        Thread.sleep(1000);
        blockIntake.stopSucking();
        strafe.left(0.2, 24);
        gyroTurn.absolute(0);
        blockIntake.kickBlock();
        drive.forward(0.2, 70);
        blockFlipper.liftOut();
    }
}
