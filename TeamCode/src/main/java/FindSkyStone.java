import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class FindSkyStone {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    Drive drive;

    public FindSkyStone (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        this.drive = new Drive(robot, telemetry, linearOpMode);
    }
    public boolean preliminary(){
        if(getAdjustedRed() < threshold(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)))
            return true;
        return false;
    }
    public int instant(AllianceColor alliance) throws InterruptedException {
        int red = getAdjustedRed();
        int leftRed = getLeftAdjustedRed();
        int avg = (red+leftRed)/2;
        double frac;
        if(red < leftRed){
            frac = (float) red/avg;
            if(alliance == AllianceColor.Red){
                if(frac < robot.THRESHOLD_PERCENT) {
                    telemetry.addData("Red Fraction", frac);
                    telemetry.addData("BlockNum", 2);
                    telemetry.update();
                    return 2;
                }
                else{
                    telemetry.addData("Red Fraction", frac);
                    telemetry.addData("BlockNum", 1);
                    telemetry.update();
                    return 1;
                }
            }
            else{
                if(frac < robot.THRESHOLD_PERCENT)
                    return 2;
                else
                    return 3;
            }
        }
        else {
            frac = (float) leftRed/avg;
            telemetry.addData("Left Red Fraction", frac);
            telemetry.update();
            if (alliance == AllianceColor.Red) {
                if (frac < robot.THRESHOLD_PERCENT)
                    return 3;
                else
                    return 1;
            }
            else {
                if (frac < robot.THRESHOLD_PERCENT)
                    return 1;
                else
                    return 3;
            }
        }
    }
    private double threshold (double distance) {
        return (Math.pow(distance, -1.247)) * 3003;
    }
    private int getAdjustedRed () { return robot.frontColorSensor.red();}
    private int getLeftAdjustedRed () { return robot.frontLeftColorSensor.red();}
}