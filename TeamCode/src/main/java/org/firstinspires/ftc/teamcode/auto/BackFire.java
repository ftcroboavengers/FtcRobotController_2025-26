package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "back + Fire (Pinpoint)", group = "RoboAvengers")
public class BackFire extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;
    private GoBildaPinpointDriver pinpoint;

    // Motion tuning
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MAX_TURN_POWER = 0.4;

    // Simple proportional gains (tune these)
    private static final double kDrive = 0.02;   // position → power
    private static final double kTurn  = 0.008;  // heading error → power

    // Launcher
    private static final double LAUNCH_TARGET = 1200;

    // State machine
    private enum AutoState { FORWARD, TURN, FIRE, STOP, DONE, SET }
    private AutoState state = AutoState.FORWARD;

    // Stability counter for turning
    private int stableCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Map hardware ---
        leftFront  = firstMotor("front_left_drive",  "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",   "backLeftMotor");
        rightBack  = firstMotor("back_right_drive",  "backRightMotor");
        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // Drive directions
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // --- Launcher setup ---
        if (leftLauncher != null) {
            leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (rightLauncher != null) {
            rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // --- Pinpoint setup ---
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            telemetry.addLine("Pinpoint not found!");
            telemetry.update();
            sleep(2000);
            pinpoint = null;
        }

        telemetry.addLine("READY: Pinpoint Auto");
        telemetry.update();

        waitForStart();

        // --- State machine loop ---
        while (opModeIsActive() && state != AutoState.DONE) {
            if (pinpoint != null) pinpoint.update();

            switch (state) {
                case FORWARD:
                    telemetry.addLine("State: Back 48 in");
                    if (moveToX(-48.0)) {
                        stopDrive();
                        state = AutoState.FIRE;
                    }
                    break;

                case FIRE:
                    telemetry.addLine("State: FIRE (5s)");
                    startLaunchers();
                    sleep(300);
                    if (intake != null) intake.setPower(1.0);
                    sleep(5000);
                    if (intake != null) intake.setPower(0);
                    stopLaunchers();
                    state = AutoState.STOP;
                    break;

                case STOP:
                    telemetry.addLine("State: STOP");
                    stopDrive();
                    state = AutoState.DONE;
                    break;

                default:
                    state = AutoState.DONE;
                    break;
            }

            // Telemetry feedback
            if (pinpoint != null) {
                telemetry.addData("X (in)", "%.1f", pinpoint.getPosX(DistanceUnit.INCH));
                telemetry.addData("Y (in)", "%.1f", pinpoint.getPosY(DistanceUnit.INCH));
                telemetry.addData("Heading (deg)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));
            }

            if (leftLauncher != null && rightLauncher != null) {
                telemetry.addData("Left launcher vel", "%.0f", leftLauncher.getVelocity());
                telemetry.addData("Right launcher vel", "%.0f", rightLauncher.getVelocity());
            }

            telemetry.update();
            idle();
        }

        stopLaunchers();
        stopDrive();
        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }

    // ---------------- Pinpoint control helpers ----------------

    // Drive forward until we reach a target X distance (inches)
    private boolean moveToX(double targetXInches) {
        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double error = targetXInches - currentX;
        double power = kDrive * error;

        power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, power));
        setPower(power, power, power, power);

        return Math.abs(error) < 1.0;  // stop within 1 inch
    }

    // Turn to an absolute heading (degrees)
    private boolean turnToHeading(double targetDeg) {
        double current = normalize180(pinpoint.getHeading(AngleUnit.DEGREES));
        double error = normalize180(targetDeg - current);

        double power = kTurn * error;

        // Minimum power to overcome friction
        if (Math.abs(power) < 0.15 && Math.abs(error) > 3) {
            power = Math.copySign(0.15, power);
        }

        // Clip
        power = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, power));

        // Flip for reversed left motors
        setPower(-power, power, -power, power);

        boolean onTarget = Math.abs(error) < 3.0;
        if (onTarget) stableCount++;
        else stableCount = 0;

        return stableCount > 5;  // must be stable for a few cycles
    }

    private double normalize180(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    // ---------------- Utility methods ----------------

    private void startLaunchers() {
        if (leftLauncher != null) leftLauncher.setVelocity(LAUNCH_TARGET);
        if (rightLauncher != null) rightLauncher.setVelocity(LAUNCH_TARGET);
    }

    private void stopLaunchers() {
        if (leftLauncher != null) leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront  != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack   != null) leftBack.setPower(bl);
        if (rightBack  != null) rightBack.setPower(br);
    }

    private void stopDrive() {
        setPower(0, 0, 0, 0);
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) {
            if (m != null) m.setZeroPowerBehavior(BRAKE);
        }
    }

    private DcMotor firstMotor(String primary, String alt) {
        DcMotor m = getMotor(primary);
        if (m == null) m = getMotor(alt);
        return m;
    }

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }

    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); }
        catch (Exception e) { return null; }
    }
}
