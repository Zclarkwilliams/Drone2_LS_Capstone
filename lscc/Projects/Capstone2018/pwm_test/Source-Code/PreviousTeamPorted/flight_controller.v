/*

Code from ECE412/3 Capstone team from 2017, Lucas Myers,  Svyatoslav Zhuchenya, Casey Montgomery, Charley  Hill
File downloaded from: https://github.com/lucas709/drone_soc/blob/master/verilog_src/flight_controller.v

*/


/*
Assigned pins on MachX03LF board as follows:

pitch_pwm B5 
roll_pwm E6 
yaw_pwm E8 
throttle_pwm A5 
switch_pwm C4 
turret_rot_pwm_in  E9 
turret_elev_pwm_in B12
turret_elev_gnd    I01 WiredGndPin <- Note: This one has to  be wired in

motor1_pwm D16
motor1_5v  E16 Wired5vPin
motor2_pwm H16
motor2_5v  J16 Wired5vPin
motor3_pwm K14 
motor3_5v  L15 Wired5vPin
motor4_pwm C15 
motor4_5v  E14 Wired5vPin
turret_rot_pwm_out G15 
turret_rot_5v      H15 Wired5vPin
turret_elev_pwm_out K16 
turret_elev_5v      L16 Wired5vPin

LED1 -> LatticeName LED0 H11 
LED2 -> LatticeName LED1 J13
LED3 -> LatticeName LED2 J11
LED4 -> LatticeName LED3 L12
LED5 -> LatticeName LED4 K11
LED6 -> LatticeName LED5 L13
LED7 -> LatticeName LED6 N15
LED8 -> LatticeName LED7 P16
*/
module flight_controller(motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm, led, turret_rot_pwm_out, turret_elev_pwm_out, pitch_pwm, roll_pwm, yaw_pwm, throttle_pwm, switch_pwm, turret_rot_pwm_in, turret_elev_pwm_in);
   
   output motor1_pwm;
   output motor2_pwm;
   output motor3_pwm;
   output motor4_pwm;
   output [7:0]led;
   output turret_rot_pwm_out;
   output turret_elev_pwm_out;
   input wire pitch_pwm;
   input wire roll_pwm;
   input wire yaw_pwm;
   input wire throttle_pwm;
   input wire switch_pwm;
   input turret_rot_pwm_in;
   input turret_elev_pwm_in;
   
   wire         clk;
   wire [9:0]   w1, w2, w3, w4, w25;
   wire [10:0]  w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23, w24;
   wire         m1, m2, m3, m4;

   clock clock_inst(clk);
   
   reciever_reader pitch_reader(clk, pitch_pwm, w1);
   reciever_reader roll_reader(clk, roll_pwm, w2);
   reciever_reader yaw_reader(clk, yaw_pwm, w3);
   reciever_reader throttle_reader(clk, throttle_pwm, w4);
   reciever_reader switch_reader(clk, switch_pwm, w25);
   
   /*output the offset for the respective control to motor 1 on the first wire, motor 2 on the second wire
    , motor 3 on the third wire, and motor 4 on the fourth wire.*/
   pitch_offset_generator pitch_offset_generator_inst(w5, w6, w7, w8, w1, w4/*throttle offset*/, clk);
   roll_offset_generator roll_offset_generator_inst(w9, w10, w11, w12, w2, w4/*throttle offset*/, clk);
   yaw_offset_generator yaw_offset_generator_inst(w13, w14, w15, w16, w3, w4/*throttle offset*/, clk);
   throttle_offset_generator throttle_offset_generator_inst(w17, w18, w19, w20, led, w4, w25, clk);

   motor_offset_summer motor_1_offset_summer(w21/*output offset of motor 1 to w21*/, w5, w9, w13, w17, clk);
   motor_offset_summer motor_2_offset_summer(w22/*output offset of motor 2 to w22*/, w6, w10, w14, w18, clk);
   motor_offset_summer motor_3_offset_summer(w23/*output offset of motor 3 to w23*/, w7, w11, w15, w19, clk);
   motor_offset_summer motor_4_offset_summer(w24/*output offset of motor 3 to w24*/, w8, w12, w16, w20, clk);
   
   pwm_generator motor1_pwm_generator(motor1_pwm, w21, clk);
   pwm_generator motor2_pwm_generator(motor2_pwm, w22, clk);
   pwm_generator motor3_pwm_generator(motor3_pwm, w23, clk);
   pwm_generator motor4_pwm_generator(motor4_pwm, w24, clk);
   
   assign turret_rot_pwm_out = turret_rot_pwm_in; //Pass camera turret rotation PWM input to output
   assign turret_elev_pwm_out = turret_elev_pwm_in;//Pass camera turret elevation PWM input to output

endmodule