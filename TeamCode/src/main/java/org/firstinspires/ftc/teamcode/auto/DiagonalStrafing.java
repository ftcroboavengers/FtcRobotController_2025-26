package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="Diagonal Strafing", group="DriveAuton")
public class DiagonalStrafing extends LinearOpMode
//@Disabled
{
    // Define Motors and Servos in OpMode
    public DcMotor  leftFrontDrive   = null; //the left front drivetrain motor
    public DcMotor  rightFrontDrive  = null; //the right drivetrain motor
    public DcMotor  leftBackDrive    = null; //the left back drivetrain motor
    public DcMotor  rightBackDrive   = null; //the right back drivetrain motor
    public DcMotor  armMotor         = null; //the arm motor
    public DcMotor  slideMotor       = null; //the slider
    public Servo    claw             = null; //the claw servo
    public Servo    clawHead         = null; //the claw head servo

    // Declare contants
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 10 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 70 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 25 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_HIGH_BASKET     = 100 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_HIGH_BASKET2     = 100 * ARM_TICKS_PER_DEGREE;
    final double LIFT_TICKS_PER_MM = 537.7 / 120.0;
    final double LIFT_SCORING_IN_HIGH_BASKET = 475 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_SAMPLE = 145 * LIFT_TICKS_PER_MM;
   // Changing the speed you run the motors at affects the slippage
    static final double FORWARD_SPEED         = 0.4;
    static final double TURN_SPEED            = 0.5;
    static final double STRAFE_SPEED          = 0.4;
    static final double DIAGONAL_STRAFE_SPEED = 0.4;
    static final double ZERO_SPEED            = 0.0;
    final double CLAW_OPEN   = 0.0;
    final double CLAW_CLOSED  = 1.0;


    /* Variables that are used to set the arm to a specific position */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.dcMotor.get("frontLeftMotor");
        leftBackDrive = hardwareMap.dcMotor.get("backLeftMotor");
        rightFrontDrive = hardwareMap.dcMotor.get("frontRightMotor");
        rightBackDrive = hardwareMap.dcMotor.get("backRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        claw = hardwareMap.get(Servo.class, "claw");
        clawHead = hardwareMap.get(Servo.class, "clawHead");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        //Step 1: Diagonal Strafe: Top Left
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); //Run
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); //Run
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setPower(ZERO_SPEED);
        leftBackDrive.setPower(DIAGONAL_STRAFE_SPEED);
        rightFrontDrive.setPower(DIAGONAL_STRAFE_SPEED);
        rightBackDrive.setPower(ZERO_SPEED);
        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < 1000)) {
            telemetry.addData("Path", "Fwd Drive 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Step 1: Drive Forward: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();


        //Step 2: Diagonal Strafe: Top Right
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); //Run
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD); //Run

        leftFrontDrive.setPower(DIAGONAL_STRAFE_SPEED);
        leftBackDrive.setPower(ZERO_SPEED);
        rightFrontDrive.setPower(ZERO_SPEED);
        rightBackDrive.setPower(DIAGONAL_STRAFE_SPEED);

        while (opModeIsActive() && (runtime.milliseconds() < 1000))
        {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Step 2: Strafe right: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        //Step 3: Diagonal Strafe: Bottom Left
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); //Run
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE); //Run

        leftFrontDrive.setPower(DIAGONAL_STRAFE_SPEED);
        leftBackDrive.setPower(ZERO_SPEED);
        rightFrontDrive.setPower(ZERO_SPEED);
        rightBackDrive.setPower(DIAGONAL_STRAFE_SPEED);

        while (opModeIsActive() && (runtime.milliseconds() < 1000))
        {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Step 3: Drive backwards: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        //Step 4: Diagonal Strafe: Bottom Right
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); //Run
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); //Run
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setPower(ZERO_SPEED);
        leftBackDrive.setPower(DIAGONAL_STRAFE_SPEED);
        rightFrontDrive.setPower(DIAGONAL_STRAFE_SPEED);
        rightBackDrive.setPower(ZERO_SPEED);

        while (opModeIsActive() && (runtime.milliseconds() < 1000))
        {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Step 4: Return to original position: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

    }

}
