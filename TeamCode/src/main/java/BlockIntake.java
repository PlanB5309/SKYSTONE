import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlockIntake {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public BlockIntake (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    enum BLOCK_KICK {
        KICKER_IN,
        BLOCK_GRABBED,
        KICKER_OUT
    }
    BLOCK_KICK current_state = BLOCK_KICK.KICKER_OUT;
    Long startTime;

    public void startSucking () {
        robot.blockKickerServo.setPosition(robot.KICKER_OUT_POSITION);
        robot.leftIntakeMotor.setPower(robot.INTAKE_WHEEL_SPEED);
        robot.rightIntakeMotor.setPower(robot.INTAKE_WHEEL_SPEED);
    }

    public void stopSucking () {
        robot.leftIntakeMotor.setPower(0);
        robot.rightIntakeMotor.setPower(0);
        robot.blockKickerServo.setPosition(robot.KICKER_STANDARD_POSITION);
    }

    public void runBlockKicker () throws InterruptedException{
        if (current_state == BLOCK_KICK.KICKER_OUT) {
            startTime = System.currentTimeMillis();
            current_state = Block_Kicker(current_state, startTime);
        }
        if (current_state != BLOCK_KICK.KICKER_OUT) {
            current_state = Block_Kicker(current_state, startTime);
        }
    }

    private BLOCK_KICK Block_Kicker (BLOCK_KICK current_state, Long startTime) throws InterruptedException{
        switch (current_state) {
            case KICKER_OUT:
                robot.blockKickerServo.setPosition(robot.KICKER_IN_POSITION);
                return BLOCK_KICK.KICKER_IN;
            case KICKER_IN:
                if (System.currentTimeMillis() > (startTime + 500)) {
                    robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_GRAB);
                    return BLOCK_KICK.BLOCK_GRABBED;
                }
                break;
            case BLOCK_GRABBED:
                if (System.currentTimeMillis() > (startTime + 1000)) {
                    robot.blockKickerServo.setPosition(robot.KICKER_STANDARD_POSITION);
                    return BLOCK_KICK.KICKER_OUT;
                }
                break;
        }
        return current_state;
    }
}
