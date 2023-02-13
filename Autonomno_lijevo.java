

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="autonomno lijevo-76cm", group="Robot")

public class Autonomno_lijevo extends LinearOpMode {
  
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
    
    ColorSensor color;
    
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
        left1.setDirection(DcMotor.Direction.REVERSE);
        right1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.FORWARD);
        //
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
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
        arm_r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        
        color = hardwareMap.get(ColorSensor.class, "Color");
        
        waitForStart();
        runtime.reset();
 
        //varibales dedicated to holding the target position for encoderss dedicated to the arm
        int target_a = 0;
        int target_b = 0;
        int target_c = 0;
        int target_r = 0;
        //the number of encoder counts per rad for the gear ration on the arm
        double count_per_rad = 372.5;
        //staarting angle of each joint compared to the ground
        double angle_a_nom = Math.PI/2;
        double angle_b_nom = Math.PI/2;
        double angle_c_nom = Math.PI;
        //variabele holding the current angle of the hand. Later changed by the gamepad2
        double angle_c = angle_c_nom;
        //the lenghts of the individual segments of the arm in mm
        double a = 337;
        double b = 330;
        double c = 190;
        //forward kinematics for the starting coordinate
        double pos_x_0_target = Math.cos(angle_a_nom) * a + Math.cos(angle_b_nom) * b + Math.cos(angle_c_nom) * c;
        double pos_y_0_target = Math.sin(angle_a_nom) * a + Math.sin(angle_b_nom) * b + Math.sin(angle_c_nom) * c;
        //adition variable needed for interpolation
        double pos_x_0 = pos_x_0_target;
        double pos_y_0 = pos_y_0_target;
        //variabele used to run a specific code once
        int one_time_run = 0;
        int steps = 0;
        //angles compared to the previous arm segmet. In the case of the first arm segment comparted to the ground
        double angle_a_offset = Math.PI/2;
        double angle_b_offset = Math.PI;
        double angle_c_offset = Math.PI*1.5;
        //variables storing the previous angle value for angle correction
        double angle_a_old = angle_a_nom;
        double angle_b_old = angle_b_nom;
        double angle_c_old = angle_c_nom;
        //trigger for holding the cone on the start
        boolean trigger = false;
        //variable to decide witch solution of the inverse kinematics to use
        boolean state = true;
        boolean interp_state_x = false;
        boolean interp_state_y = false;
        double time = 0;
        //take gamepad input on interval
        int[] cone_color = new int[3];
        double input_time = 0;
        boolean read = false;
        double distance = 39;
        boolean relese_q1 = false;
        boolean relese_q2 = false;
        double integral_error_left1 = 0;
        double integral_error_left2 = 0;
        double integral_error_right1 = 0;
        double integral_error_right2 = 0;
        double integral_error_r = 0;
        double integral_error_a = 0;
        double integral_error_b = 0;
        double integral_error_c = 0;
        int error_r = 0;
        int error_a = 0;
        int error_b = 0;
        int error_c = 0;
        int error_left1 = 0;
        int error_left2 = 0;
        int error_right1 = 0;
        int error_right2 = 0;
        boolean ready_l1 = false;
        boolean ready_l2 = false;
        boolean ready_r1 = false;
        boolean ready_r2 = false;
        boolean ready_r = false;
        boolean ready_a = false;
        boolean ready_b = false;
        boolean ready_c = false;
        int target_a_unfold = 0;
        int target_b_unfold = 0;
        int target_c_unfold = 0;
        boolean ready_kinematics = false;
        double count_per_cm = 12;
        int target_l1 = (int)(count_per_cm * distance);;
        int target_l2 = (int)(count_per_cm * distance);;
        int target_r1 = (int)(count_per_cm * distance);;
        int target_r2 = (int)(count_per_cm * distance);;
        double[] resoult = new double[3];
        double kp = 0.045;
        double ki = 0.00000005;
        double kd = 0.00016;
        double kp_arm = 0.031;
        double ki_arm = 0.00000003;
        double kd_arm = 0;
                        
        if (opModeIsActive()) {
            input_time = runtime.seconds();    
            while (opModeIsActive()) {

                resoult = PID_control(target_l2,time,integral_error_left2,left2.getCurrentPosition(),error_left2,kp,ki,kd);
                left2.setPower(resoult[1]);
                integral_error_left2 = resoult[0];
                error_left2 = (int)resoult[2];
                //telemetry.addData("PIDl2", resoult[1] + "   " + error_left2  + "   " + left2.getCurrentPosition());
                if (Math.abs(error_left2) < 5) {
                     ready_l2 = true;
                }
                else {
                     ready_l2 = false;
                }

                resoult = PID_control(target_r1,time,integral_error_right1,right1.getCurrentPosition(),error_right1,kp,ki,kd);
                right1.setPower(resoult[1]);
                integral_error_right1 = resoult[0];
                error_right1 = (int)resoult[2];
                //telemetry.addData("PIDr1", resoult[1] + "   " + error_right1  + "   " + right1.getCurrentPosition());
                if (Math.abs(error_right1) < 5) {
                     ready_r1 = true;
                }
                else {
                     ready_r1 = false;
                }
                
                resoult = PID_control(target_r2,time,integral_error_right2,right2.getCurrentPosition(),error_right2,kp,ki,kd);
                right2.setPower(resoult[1]);
                integral_error_right2 = resoult[0];
                error_right2 = (int)resoult[2];
                //telemetry.addData("PIDr2", resoult[1] + "   " + error_right2  + "   " + right2.getCurrentPosition());
                if (Math.abs(error_right2) < 5) {
                     ready_r2 = true;
                }
                else {
                     ready_r2 = false;
                }
                
                resoult = PID_control(target_l1,time,integral_error_left1,left1.getCurrentPosition(),error_left1,kp,ki,kd);
                left1.setPower(resoult[1]);
                integral_error_left1 = resoult[0];
                error_left1 = (int)resoult[2];
                //telemetry.addData("PIDl1", resoult[1] + "   " + error_left1 + "   " + left1.getCurrentPosition());
                if (Math.abs(error_left1) < 5) {
                     ready_l1 = true;
                }
                else {
                     ready_l1 = false;
                }
                

                
                resoult = PID_control(-target_r,time,integral_error_r,arm_r.getCurrentPosition(),error_r,kp-0.02,ki,kd+0.05);
                arm_r.setPower(resoult[1]);
                integral_error_r = resoult[0];
                error_r = (int)resoult[2];
                //telemetry.addData("PID_R", resoult[1] + "   " + error_r  + "   " + arm_r.getCurrentPosition());
                if (Math.abs(error_r) < 2) {
                     ready_r = true;
                }
                else {
                     ready_r = false;
                }
                
                resoult = PID_control(target_a + target_a_unfold,time,integral_error_a,arm_1.getCurrentPosition(),error_a,kp_arm,ki_arm,kd_arm);
                arm_1.setPower(resoult[1]);
                integral_error_a = resoult[0];
                error_a = (int)resoult[2];
                if (Math.abs(error_a) < 6) {
                     ready_a = true;
                }
                else {
                     ready_a = false;
                }
                
                resoult = PID_control(target_b + target_b_unfold,time,integral_error_b,arm_2.getCurrentPosition(),error_b,kp_arm,ki_arm,kd_arm);
                arm_2.setPower(resoult[1]);
                integral_error_b = resoult[0];
                error_b = (int)resoult[2];
                if (Math.abs(error_b) < 6) {
                     ready_b = true;
                }
                else {
                     ready_b = false;
                }
                
                resoult = PID_control(target_c + target_c_unfold,time,integral_error_c,arm_3.getCurrentPosition(),error_c,kp_arm,ki_arm,kd_arm);
                arm_3.setPower(resoult[1]);
                integral_error_c = resoult[0];
                error_c = (int)resoult[2];
                if (Math.abs(error_c) < 6) {
                     ready_c = true;
                }
                else {
                     ready_c = false;
                }
                
                time = runtime.milliseconds();
                
                if      (steps == 0 ) {
                    steps = 1;
                    target_a_unfold = 1000;
                    servo.setPosition(1);
                 }
                else if (steps == 1 && ready_l1 && ready_l2 && ready_r1 && ready_r2) {
                    for (int n = 0; n < 10; n++) {
                        cone_color[0] += color.red();
                        cone_color[1] += color.green();
                        cone_color[2] += color.blue();
                        sleep(10);
                    }
                    cone_color[0] = cone_color[0] / 10;
                    cone_color[1] = cone_color[1] / 10;
                    cone_color[2] = cone_color[2] / 10;
                    steps = 2;
                    telemetry.addData("RGB:", cone_color[0] + "," + cone_color[1] + "," + cone_color[2]);
                    telemetry.update();
                    read = true;
                    //step2
                    distance += 85;
                    target_l1 = (int)(count_per_cm * distance);
                    target_l2 = (int)(count_per_cm * (distance + 5));
                    target_r1 = (int)(count_per_cm * distance);
                    target_r2 = (int)(count_per_cm * distance);
                    
                }
                else if (steps == 2 && ready_a) {
                    steps = 3;
                   // arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                else if (steps == 3) {
                    steps = 4;
                    target_b_unfold = -1010;
                    target_c_unfold = 1400;
                }
                else if (steps == 4 && ready_b) {
                    steps = 5;
                }
                else if (steps == 5 && ready_c) {
                    steps = 6;
                }
                else if (steps == 6) {
                    steps = 7;
                    target_r = -235;

                    ready_r = false;
                }
                else if (steps == 7 && ready_r) {
                    steps = 8;
                    pos_x_0_target = -330;
                    pos_y_0_target = 600;
                    angle_c = Math.PI;
                    interp_state_x = true;
                    interp_state_y = true;
                    ready_kinematics = true;
                }
                else if (steps == 8 && relese_q1 && relese_q2 && (runtime.seconds() - input_time) > 1) {
                    steps = 9;
                    servo.setPosition(0.1);
                    input_time = runtime.seconds();
                }
                else if (steps == 9 && (runtime.seconds() - input_time) > 1) {
                    steps = 10;
                    pos_x_0_target = Math.cos(angle_a_nom) * a + Math.cos(angle_b_nom) * b + Math.cos(angle_c_nom) * c;
                    pos_y_0_target = Math.sin(angle_a_nom) * a + Math.sin(angle_b_nom) * b + Math.sin(angle_c_nom) * c;
                    interp_state_x = true;
                    interp_state_y = true;
                    relese_q1 = false;
                    relese_q2 = false;
                    target_l1 = (int)(count_per_cm * (distance-38));
                    target_l2 = (int)(count_per_cm * (distance+38));
                    target_r1 = (int)(count_per_cm * (distance+38));
                    target_r2 = (int)(count_per_cm * (distance-38));
                }
                else if (steps == 10 && relese_q1 && relese_q2) {
                    steps = 11;
                    target_r = 380;
                }
                else if (steps == 11 && ready_r) {
                    steps = 12;
                    pos_x_0_target = -560;
                    pos_y_0_target = 0;
                    angle_c = Math.PI;
                    interp_state_x = true;
                    interp_state_y = true;
                    relese_q1 = false;
                    relese_q2 = false;
                }
                else if (steps == 12 && relese_q1 && relese_q2) {
                    steps = 13;
                    pos_x_0_target = -560;
                    pos_y_0_target = -130;
                    angle_c = Math.PI;
                    interp_state_x = true;
                    interp_state_y = true;
                    relese_q1 = false;
                    relese_q2 = false;
                    
                }
                else if (steps == 13 && relese_q1 && relese_q2) {
                    steps = 14;
                    servo.setPosition(0.9);
                    input_time = runtime.seconds();
                }
                else if (steps == 14 && (runtime.seconds() - input_time) > 2) {
                    steps = 15;
                    pos_x_0_target = -560;
                    pos_y_0_target = 0;
                    angle_c = Math.PI;
                    interp_state_x = true;
                    interp_state_y = true;
                    relese_q1 = false;
                    relese_q2 = false;
                    
                }
                else if (steps == 15 && relese_q1 && relese_q2) {
                    steps = 16;
                    pos_x_0_target = Math.cos(angle_a_nom) * a + Math.cos(angle_b_nom) * b + Math.cos(angle_c_nom) * c;
                    pos_y_0_target = Math.sin(angle_a_nom) * a + Math.sin(angle_b_nom) * b + Math.sin(angle_c_nom) * c;
                    interp_state_x = true;
                    interp_state_y = true;
                    relese_q1 = false;
                    relese_q2 = false;
                    target_l1 = (int)(count_per_cm * (distance));
                    target_l2 = (int)(count_per_cm * (distance));
                    target_r1 = (int)(count_per_cm * (distance));
                    target_r2 = (int)(count_per_cm * (distance));
                }
                else if (steps == 16 && relese_q1 && relese_q2) {
                  steps = 17;
                  target_r = -252;
                }
                else if (steps == 17 && ready_r) {
                    steps = 18;
                    pos_x_0_target = -320;
                    pos_y_0_target = 610;
                    angle_c = Math.PI;
                    interp_state_x = true;
                    interp_state_y = true;
                    relese_q1 = false;
                    relese_q2 = false;
                    input_time = runtime.seconds();
                }
                else if (steps == 18 && relese_q1 && relese_q2 && (runtime.seconds() - input_time) > 1) {
                    steps = 19;
                    servo.setPosition(0.1);
                    input_time = runtime.seconds();
                }
                else if (steps == 19 && (runtime.seconds() - input_time) > 1) {
                    steps = 20;

                    if (cone_color[0] > cone_color[1] && cone_color[0] > cone_color[2]) {
                        //red  
                        target_l1 = (int)(count_per_cm * (distance-70));
                        target_l2 = (int)(count_per_cm * (distance+70));
                        target_r1 = (int)(count_per_cm * (distance+70));
                        target_r2 = (int)(count_per_cm * (distance-70));
        
                    }
                    else if (cone_color[1] > cone_color[0] && cone_color[1] > cone_color[2]) {
                        //green
                        //do nothing
                    }
                    else if (cone_color[2] > cone_color[0] && cone_color[2] > cone_color[1]) {
                        //blue
                        target_l1 = (int)(count_per_cm * (distance+70));
                        target_l2 = (int)(count_per_cm * (distance-70));
                        target_r1 = (int)(count_per_cm * (distance-70));
                        target_r2 = (int)(count_per_cm * (distance+70));

                    }
                    
                }
                else if (steps == 20) {
                    servo.setPosition(0.5);
                    pos_x_0_target = Math.cos(angle_a_nom) * a + Math.cos(angle_b_nom) * b + Math.cos(angle_c_nom) * c;
                    pos_y_0_target = Math.sin(angle_a_nom) * a + Math.sin(angle_b_nom) * b + Math.sin(angle_c_nom) * c;
                    //target_a_unfold = 0;
                    //target_b_unfold = 0;
                    //target_c_unfold = 0;
                    angle_c = Math.PI;
                    interp_state_x = true;
                    interp_state_y = true;
                    target_r = 0;
                }
                if (ready_kinematics) {    
                
                double interp_x = 4;
                double interp_y = 5;
                
                
                if (((pos_x_0 - pos_x_0_target) <= interp_x) && interp_state_x == true && ((pos_x_0 - pos_x_0_target) < 0)) {
                    pos_x_0 += interp_x;    
                } 
                else if (((pos_x_0 - pos_x_0_target) >= interp_x)  && interp_state_x == true && ((pos_x_0 - pos_x_0_target) > 0)) {
                    pos_x_0 -= interp_x;
                }
                else if (interp_state_x == true){
                    pos_x_0 = pos_x_0_target;
                    interp_state_x = false;
                    relese_q1 = true;
                    input_time = runtime.seconds();
                }
                
                if (((pos_y_0 - pos_y_0_target) <= interp_y)  && interp_state_y == true && ((pos_y_0 - pos_y_0_target) < 0)) {
                    pos_y_0 += interp_y;    
                } 
                else if (((pos_y_0 - pos_y_0_target) >= interp_y)  && interp_state_y == true && ((pos_y_0 - pos_y_0_target) > 0)) {
                    pos_y_0 -= interp_y;
                }
                else if (interp_state_y == true){
                    pos_y_0 = pos_y_0_target;
                     interp_state_y = false;
                     relese_q2 = true;
                     input_time = runtime.seconds();
                }
                  
                double pos_x_1 = pos_x_0 - Math.cos(angle_c) * c;
                double pos_y_1 = pos_y_0 - Math.sin(angle_c) * c;
                
                double lenght_ab = Math.sqrt(Math.pow(pos_x_1, 2) + Math.pow(pos_y_1, 2));
                
                double angle_a_1 = Math.acos((Math.pow(a, 2) + Math.pow(lenght_ab, 2) - Math.pow(b, 2)) / (2 * a * lenght_ab));
                double angle_a_2 = Math.atan2(pos_y_1,pos_x_1);
                
                double angle_a = angle_a_nom;
                //deciding the solution
    
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
                /* //in case There is a intersection between 2 segments
                if ((angle_a < (Math.PI/5) && angle_a > (-Math.PI/5)) || ((Math.PI - angle_a + angle_b) < (Math.PI/5) && (Math.PI - angle_a + angle_b) > (-Math.PI/5)) || ((Math.PI - angle_b + angle_c) < (Math.PI/3) && (Math.PI - angle_b + angle_c) > (Math.PI/3))){
                    angle_a = angle_a_old;
                    angle_b = angle_b_old;
                    angle_c = angle_c_old;
                    red.setState(true);
                }
                else {
                    red.setState(false);
                } */
                
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
                
                
                
                /*
                //set target position
                arm_1.setTargetPosition(target_a);
                arm_2.setTargetPosition(target_b);
                arm_3.setTargetPosition(target_c);
                //arm_r.setTargetPosition(target_r);
                //run this code one time
                if (one_time_run == 0) {
                one_time_run = 1;
                arm_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_1.setPower(1);
                arm_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_2.setPower(1);
                arm_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_3.setPower(1);
                //arm_r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //arm_r.setPower(1);
                } */
          /*    telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Position:", "\n X = " + pos_x_0 + "\n Y = " + pos_y_0);
                telemetry.addData("Forward:", "\n X = " + (Math.cos(angle_a) * a + Math.cos(angle_b) * b + Math.cos(angle_c) * c) + "\n Y = " + (Math.sin(angle_a) * a + Math.sin(angle_b) * b + Math.sin(angle_c) * c));
              telemetry.addData("Angles", "\n Angle_c = " + Math.toDegrees(angle_c) + "\n Angle_b = " + Math.toDegrees(angle_b) + "\n Angle_a = " + Math.toDegrees(angle_a));
                telemetry.addData("Encoder", arm_1.getCurrentPosition() + " " + arm_2.getCurrentPosition() + " " + arm_3.getCurrentPosition() + " " + arm_r.getCurrentPosition());
                telemetry.addData("Targets", (target_a + target_a_unfold) +  " " + (target_b + target_b_unfold) + " " + (target_c + target_c_unfold));
                telemetry.update();
          */          
            

                
                
            }
             }
        }
    }
    
     public double[] PID_control (int target, double current_time, double integral_error,int current_count, int error_old, double kp,double ki, double kd) {
    
    
    int error = target - current_count;
    integral_error += error * (runtime.milliseconds() - current_time);
    double derivative_error = (error_old - error) / (runtime.milliseconds() - current_time);
    double power = kp * error + ki * integral_error + kd * derivative_error;
    double[] resoult = new double[3];
    resoult[0] = integral_error;
    resoult[1] = power;
    resoult[2] = error;
    return resoult;
    
    
}
}
