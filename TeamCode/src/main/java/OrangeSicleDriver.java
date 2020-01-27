
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="OrangeSicle")
public class OrangeSicleDriver extends LinearOpMode {
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
/*    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    BlockIntake blockIntake = new BlockIntake(robot, telemetry, this);
    BlockFlipper blockFlipper = new BlockFlipper(robot, telemetry, this);
*/
    StopAtDistance stopAtDistance = new StopAtDistance(robot, telemetry, this);
    BlockArm blockArm = new BlockArm(robot, telemetry, this);
    BlockGrabber blockGrabber = new BlockGrabber(robot, telemetry, this);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        blockArm.down();
       stopAtDistance.forward(0.1,4, 30);
       gyroTurn.absolute(0);
       int blockNum = findSkyStone.left(0.1, 16);
       gyroTurn.absolute(0);
       blockGrabber.grab();
       sleep(100);
       blockArm.up();
       stopAtDistance.left(0.1, robot.blockDistance[blockNum], 40);
       gyroTurn.absolute(0);
       stopAtDistance.left(0.1, robot.blockDistance[blockNum], 40);
    }
}
