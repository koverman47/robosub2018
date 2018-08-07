#include <Servo.h>

/*
 * P - Port
 * S - Starboard
 * F - Fore
 * A - Aft
 *
 * De - Depth
 * Fo - Forward 
 * St - Strafe
 *
 * --------------------------------
 * | Position | Pin | Orientation |
 * --------------------------------
 * |   P-Fo   |  2  |      +      |
 * |   S-Fo   |  3  |      -      |
 * |   F-St   |  4  |      -      |
 * |   A-St   |  5  |      -      |
 * |  F-P-De  |  6  |      -      |
 * |  F-S-De  |  7  |      +      |
 * |  A-P-De  |  8  |      +      |
 * |  A-S-De  |  9  |      -      |
 * --------------------------------
 *
 */


/* 8 servos for 8 thrusters */
Servo servo[8];

//* Blue Robotics T200s - PWM: 1500 +/- 400 */
int neutral = 1500;

/* Thruster command scaling */
int down = 250;
int fore = 300;
int scale = 300;

/* Pin outs */
int pins[8] = {2, 3, 4, 5, 6, 7, 8, 9}; // pin outs offset by 2 from array indices
int depthPin = 0; 

//* PID error terms */
float error = 0.0;
float dererror = 0.0;
float interror = 0.0;
float dt = 0.0;
float tim;

/* PID Gains */
float kp = 0.16; // TODO: Tune
float ki = 0.0;
float kd = 0.1;

//float target = 100;
float target;
float command = 0.0;
float measurement;


void calcUpdate() {

    /* Take depth measurement */
    measurement = analogRead(depthPin);
    
    /* update clock and change of time */
    int temp = millis();
    dt = abs(temp - tim) / 1000;
    tim = temp;
    
    /* Compute error terms */
    float preverror = error;
    error = measurement - target;
    interror += error * dt;
    dererror = (error - preverror) * dt;
}


float depthPID() {

    /* Compute unscaled PID */
    float p = (error * kp);
    float i = (interror * dt * ki);
    float d = (dererror * kd);
    command = p + i + d;

    //Serial.println(p);
    //Serial.println(i);
    //Serial.println(d);

    /* Scale command to +/- 1 */    
    if(command > 1) {
        command = 1;
    }
    else if(command < -1) {
        command = -1;
    }
    
    return command;
}

/* Print current variable values */
void printStatus() {
    Serial.print("Down: ");
    Serial.println(down);
    Serial.print("Command: ");
    Serial.println(command);
    Serial.print("Analog: ");
}


void setup() {
    Serial.begin(9600);
    
    /* Get first measurement */
    calcUpdate();
    
    /* Write initial values to thrusters */
    int i;
    for(i = 0; i < 8; i++) {
        servo[i].attach(pins[i]);
        servo[i].writeMicroseconds(neutral);
    }
    
    /* Take n samples at water surface - call average base depth */
    int samples = 50;
    for(i = 0; i < samples; i++) {
        target += analogRead(depthPin);
    }
    target = (target / samples) + 1; // offset for target depth

    /* Delay system side effects */
    delay(5000);
    
    /* Depth thrust full for initial submersion */
    servo[4].writeMicroseconds(neutral - down);
    servo[5].writeMicroseconds(neutral + down);
    servo[6].writeMicroseconds(neutral + down);
    servo[7].writeMicroseconds(neutral - down);
    delay(1000);
    
    /* Initialize timer */
    tim = millis();
}


void loop() {
    
    /* Take new measurement and compute errors */
    calcUpdate();
    
    /* Compure new command from PID */
    command = depthPID();
    
    /* Compute new depth thrust command */
    down = command * scale + neutral;

    /* Write Fore Thrust */
    servo[0].writeMicroseconds(neutral + fore);
    servo[1].writeMicroseconds(neutral - fore + 20);

    /* Write Depth Thrust */
    servo[4].writeMicroseconds(neutral - down);
    servo[5].writeMicroseconds(neutral + down);
    servo[6].writeMicroseconds(neutral + down);
    servo[7].writeMicroseconds(neutral - down);

}
