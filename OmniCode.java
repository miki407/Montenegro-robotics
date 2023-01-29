package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;




@TeleOp(name="OmniCode", group="Linear Opmode")

public class OmniCode extends LinearOpMode {
    
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1 = null;
    private DcMotor right1 = null;
    private DcMotor left2 = null;
    private DcMotor right2 = null;
    
    private DcMotor arm_r = null;
    private DcMotor arm_1 = null;
    private DcMotor arm_2 = null;
    private DcMotor arm_3 = null;
    
    Servo   servo;
    
    DigitalChannel red;
    DigitalChannel green;
    DigitalChannel blue;
    // todo: write your code here
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //map motor to config names
        left1  = hardwareMap.get(DcMotor.class, "left1");
        right1 = hardwareMap.get(DcMotor.class, "right1");
        left2  = hardwareMap.get(DcMotor.class, "left2");
        right2 = hardwareMap.get(DcMotor.class, "right2");
        //set ddirection of motors designated for movment
        left1.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.REVERSE);
        //map motors to config names
        arm_r  = hardwareMap.get(DcMotor.class, "arm_r");
        arm_1 = hardwareMap.get(DcMotor.class, "arm_1");
        arm_2  = hardwareMap.get(DcMotor.class, "arm_2");
        arm_3 = hardwareMap.get(DcMotor.class, "arm_3");
        //set direction ogf motors designated for arm movment
        arm_r.setDirection(DcMotor.Direction.FORWARD);
        arm_1.setDirection(DcMotor.Direction.FORWARD);
        arm_2.setDirection(DcMotor.Direction.REVERSE);
        arm_3.setDirection(DcMotor.Direction.REVERSE);
        //reset encoders on the motors designated for arm movment
        arm_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set motors to run using encoders 
        arm_r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //map servo to the name in the config
        servo = hardwareMap.get(Servo.class, "hand");
        //digital channels
        red = hardwareMap.get(DigitalChannel.class, "red");
        green = hardwareMap.get(DigitalChannel.class, "green");
        blue = hardwareMap.get(DigitalChannel.class, "blue");
        //set as output
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
        blue.setMode(DigitalChannel.Mode.OUTPUT);

        
        waitForStart();
        runtime.reset();
        //variables dedicated to holding the target position for encoders dedicated to movment
        int left1_target = 0;
        int right1_target = 0;
        int left2_target = 0;
        int right2_target = 0;
        //varibales dedicated to holding the target position for encoderss dedicated to the arm
        int target_a = 0;
        int target_b = 0;
        int target_c = 0;
        int target_r = 0;
        //the number of encoder counts per rad for the gear ration on the arm
        double count_per_rad = 472.5;
        //staarting angle of each joint compared to the ground
        double angle_a_nom = Math.PI/2;
        double angle_b_nom = Math.PI/2;
        double angle_c_nom = 0;
        //variabele holding the current angle of the hand. Later changed by the gamepad2
        double angle_c = angle_c_nom;
        //the lenghts of the individual segments of the arm in mm
        double a = 337;
        double b = 330;
        double c = 190;
        //forward kinematics for the starting coordinate
        double pos_x_0 = Math.cos(angle_a_nom) * a + Math.cos(angle_b_nom) * b + Math.cos(angle_c_nom) * c;
        double pos_y_0 = Math.sin(angle_a_nom) * a + Math.sin(angle_b_nom) * b + Math.sin(angle_c_nom) * c;
        //variabele used to run a specific code once
        int one_time_run = 0;
        //angles compared to the previous arm segmet. In the case of the first arm segment comparted to the ground
        double angle_a_offset = Math.PI/2;
        double angle_b_offset = Math.PI;
        double angle_c_offset = Math.PI/2;
        //variables storing the previous angle value for angle correction
        double angle_a_old = angle_a_nom;
        double angle_b_old = angle_b_nom;
        double angle_c_old = angle_c_nom;
        //trigger for holding the cone on the start
        boolean trigger = false;
        //variable to decide witch solution of the inverse kinematics to use
        boolean state = true;
        double time = 0;
        
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.5) {
            //code responsible for rotaation
                double power = gamepad1.left_stick_x;
            
                left1.setPower(power);
                right1.setPower(-power);
                left2.setPower(power);
                right2.setPower(-power);
            
            }
            else {
                //starting power 
                double r2l1 = 0.0;
                double r1l2 = 0.0;
                //getting the x and y coordinate of the stick on gamepad2
                double x = gamepad1.right_stick_x;
                double y = -gamepad1.right_stick_y;
                //calculating the power of the 
                double power = Math.sqrt( Math.pow(x , 2) + Math.pow(y , 2));
                //calculate the angle at witch we drive
                double angle = Math.atan2(x,y);
                //add a constant 45 degree angle because the power vector of the wheels are 45 degress of phase
                double angle_rel = 0.785 + angle;
                //calculate the power of individual motor pairs
                r2l1 = Math.sin(angle_rel) * power;
                r1l2 = Math.cos(angle_rel) * power;
                //set the power of the motors
                left1.setPower(r2l1);
                right1.setPower(r1l2);
                left2.setPower(r1l2);
                right2.setPower(r2l1);
                //run if the trigger is pressed

            }
            
            
            //ARM CODE
            //---------------------------------------------------------------------------------------------------
            //ARM CODE
            
            
            //every time this code runs the value of the final x and y coordinate ae updated depending on the position of the right stick
            pos_x_0 -= gamepad2.right_stick_x * 5;
            pos_y_0 -= gamepad2.right_stick_y * 5;
            //d pad buttons control the angle of the last joint(angle_c) and the target position for the rotatipn eof the base
            if (gamepad2.dpad_down == true) {
                angle_c -= 0.01; //decreasing the nagle
            }
            else if (gamepad2.dpad_up == true) {
                angle_c += 0.01; //increasing the angle
            }
            if (gamepad2.dpad_left == true) {
                target_r += 2;  //turning left
            }
            else if (gamepad2.dpad_right == true) {
                target_r -= 2;  //turning right
            }
            if (gamepad2.a == true) {
                //if the button is pressed the arm will unfold and grab the preinserted cone;
                
                servo.setPosition(0.75);
                arm_1.setTargetPosition(1000); //be upstraight
                arm_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_1.setPower(1);
                while(arm_1.isBusy()) {
                    
                }
                
                arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
                arm_1.setTargetPosition(0);
                arm_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_1.setPower(1);
                
                arm_2.setTargetPosition(-1010); //be upstraight
                arm_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_2.setPower(1);
                while(arm_2.isBusy()) {
                    
                }
                
                arm_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
                arm_2.setTargetPosition(0);
                arm_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_2.setPower(1);
                
                
                arm_3.setTargetPosition(1400); //be upstraight
                arm_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_3.setPower(1);
                while(arm_3.isBusy()) {
                }
                
                arm_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
                arm_3.setTargetPosition(0);
                arm_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_3.setPower(1);
                 
            }
            
            //controls the grabber so that it doesnt relese the preloaded cone until a input is given
            
            if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0 || (trigger == true)) {
            double servo_power = (1 + gamepad2.left_trigger - gamepad2.right_trigger) / 2; 
                servo.setPosition(servo_power);
                trigger = true;
            }
            
            //INVERSE KINEMATICS
            
            double pos_x_1 = pos_x_0 - Math.cos(angle_c) * c;
            double pos_y_1 = pos_y_0 - Math.sin(angle_c) * c;
            
            double lenght_ab = Math.sqrt(Math.pow(pos_x_1, 2) + Math.pow(pos_y_1, 2));
            
            double angle_a_1 = Math.acos((Math.pow(a, 2) + Math.pow(lenght_ab, 2) - Math.pow(b, 2)) / (2 * a * lenght_ab));
            double angle_a_2 = Math.atan2(pos_y_1,pos_x_1);
            
            double angle_a = angle_a_nom;
            //deciding the solution
            if (gamepad2.b == true && (runtime.seconds() - time) > 1){
                time = runtime.seconds();
                if (state == true) {
                    state = false;
                }
                else {
                    state = true;
                }
            }
            if (state == true) {
                angle_a = - angle_a_1 + angle_a_2; 
            }
            else {
                angle_a = angle_a_1 + angle_a_2; 
            }
            
            
            double pos_x_2 = Math.cos(angle_a) * a;
            double pos_y_2 = Math.sin(angle_a) * b;
            double pos_x_b = pos_x_1 - pos_x_2 ;
            double pos_y_b = pos_y_1 - pos_y_2;
            
            double angle_b = Math.atan2(pos_y_b, pos_x_b);
            
//-------------------------------------------------------------------------------------------------------------
            //angle pre processing
            //In case NaN value is calculated use previous correct value insted
            if (Double.isNaN(angle_a) || Double.isNaN(angle_b)) {
                angle_a = angle_a_old;
                angle_b = angle_b_old;
                angle_c = angle_c_old;
                blue.setState(true);
            }
            else {
                blue.setState(true);
            }
            //in case There is a intersection between 2 segments
            if (((angle_a - angle_a_nom) < (Math.PI/5) && (angle_a - angle_a_nom) > (-Math.PI/5)) || ((Math.PI - angle_a + angle_b - angle_b_offset) < (Math.PI/5) && (Math.PI - angle_a + angle_b - angle_b_offset) > (-Math.PI/5)) || ((Math.PI - angle_b + angle_c - angle_c_offset) < (Math.PI/3) && (Math.PI - angle_b + angle_c - angle_c_offset) > (Math.PI/3))){
                angle_a = angle_a_old;
                angle_b = angle_b_old;
                angle_c = angle_c_old;
                red.setState(true);
            }
            else {
                red.setState(false);
            }
            
            //calculating angle diffrence
            double angle_a_delta1 = Math.abs(angle_a - angle_a_old);
            double angle_a_delta2 = Math.abs(angle_a - angle_a_old + (2 * Math.PI));
            double angle_a_delta3 = Math.abs(angle_a - angle_a_old - (2 * Math.PI));
            
            double angle_b_delta1 = Math.abs(angle_b - angle_b_old);
            double angle_b_delta2 = Math.abs(angle_b - angle_b_old + (2 * Math.PI));
            double angle_b_delta3 = Math.abs(angle_b - angle_b_old - (2 * Math.PI));
            //calculating the minimum value
            double delta_a_minimum = angle_a_delta1;
            double delta_b_minimum = angle_b_delta1;
            
            if (angle_a_delta1 < angle_a_delta2) {
                delta_a_minimum = angle_a_delta1;
            }
            else {
                delta_a_minimum = angle_a_delta2;
            }
            if (delta_a_minimum < angle_a_delta3) {
                //do nothing
            }
            else {
                delta_a_minimum = angle_a_delta3;
            }
            
            if (angle_b_delta1 < angle_b_delta2) {
                delta_b_minimum = angle_b_delta1;
            }
            else {
                delta_b_minimum = angle_b_delta2;
            }
            if (delta_b_minimum < angle_b_delta3) {
                //do nothing
            }
            else {
                delta_b_minimum = angle_b_delta3;
            }
            
            //correcting the current angle to achive optimal performance
            if (delta_a_minimum == angle_a_delta1) {
                //do nothing
            }
            else if (delta_a_minimum == angle_a_delta2) {
                angle_a += 2 * Math.PI;
            }
            else if (delta_a_minimum == angle_a_delta3) {
                angle_a -= 2 * Math.PI;
            }
            
            if (delta_b_minimum == angle_b_delta1) {
                //do nothing
            }
            else if (delta_b_minimum == angle_b_delta2) {
                angle_b += 2 * Math.PI;
            }
            else if (delta_b_minimum == angle_b_delta3) {
                angle_b -= 2 * Math.PI;
            }

            angle_a_old = angle_a;
            angle_b_old = angle_b;
            angle_c_old = angle_c;
            
//----------------------------------------------------------------------------------------------------------------        

            //convert angle to encoder position
            target_a = (int)(count_per_rad * (angle_a - angle_a_nom));
            target_b = (int)(count_per_rad * (Math.PI - angle_a + angle_b - angle_b_offset));
            target_c = (int)(count_per_rad * (Math.PI - angle_b + angle_c - angle_c_offset));
             
            //set target position
            arm_1.setTargetPosition(target_a);
            arm_2.setTargetPosition(target_b);
            arm_3.setTargetPosition(target_c);
            arm_r.setTargetPosition(target_r);
            //run this code one time
            if (one_time_run == 0) {
            one_time_run = 1;
            arm_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_1.setPower(1);
            arm_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_2.setPower(1);
            arm_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_3.setPower(1);
            arm_r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_r.setPower(1);
            }
            //output current state of the robot to driver hub
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position:", "\n X = " + pos_x_0 + "\n Y = " + pos_y_0);
            telemetry.addData("Forward:", "\n X = " + (Math.cos(angle_a) * a + Math.cos(angle_b) * b + Math.cos(angle_c) * c) + "\n Y = " + (Math.sin(angle_a) * a + Math.sin(angle_b) * b + Math.sin(angle_c) * c));
            telemetry.addData("Angles", "\n Angle_c = " + Math.toDegrees(angle_c) + "\n Angle_b = " + Math.toDegrees(angle_b) + "\n Angle_a = " + Math.toDegrees(angle_a));
            telemetry.addData("Encoder", arm_1.getCurrentPosition() + " " + arm_2.getCurrentPosition() + " " + arm_3.getCurrentPosition());
            telemetry.addData("Targets", target_a + " " + target_b + " " + target_c);
            telemetry.update();
            
            //wait
            sleep(10);
            
        }
    }
    
}

