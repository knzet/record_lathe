// record lathe firmware
// built for arduino uno, Adafruit Motorshield v2 library
//
// Author: kenzie

#include <Adafruit_MotorShield.h>  // https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
#include <AccelStepper.h>          // https://github.com/adafruit/AccelStepper

// Linear Actuator hardware:
// https://github.com/uStepper/Linear_Actuator

// cutter head control
#include <Servo.h>

const unsigned char KEY_RUN = 6;
const unsigned char KEY_STOP_REW = 2;
const unsigned char KEY_TRIM_FWD = 3;
const unsigned char KEY_TRIM_BAK = 4;
const unsigned char SW_MODE = 5;
const unsigned char SERVO_PIN = 9;

// #define KEY_RUN 6
// #define KEY_STOP_REW 2
// #define KEY_TRIM_FWD 3
// #define KEY_TRIM_BAK 4
// #define SW_MODE 5
// #define SERVO_PIN 9

// defined by the radius of record, to automatically stop.
// in a later version, the music file ending playback should trigger the spiral-out. A limit switch, either optical or momentary, would be the simplest option for a hard stop.
const int FORTY_FIVE_DISTANCE = 200;    //1000
const int THIRTY_THREE_DISTANCE = 200;  //2700*8

// cutting pitch
// complete silence is 555 LPI (very slow pitch -> lines very close together, no room for amplitude)
// 0dB for a 33 is 200 LPI (faster than current setup can microstep)
// variable groove pitch is too fancy for now, since the audio is played through computer.
// potential future version could hook an opamp off the audio stream, measure rms level and speed up or slow down to give more space. Would really need finetuning.
// compromise is to cut about as fast as we can on this hardware, while keeping the smoothness of microstepping
// 217/255 is 0.52 mm/s on this hardware, which corresponds to 224 LPI at 45 RPM.
// This RPM value is CUTTING, NOT PLAYBACK. Could push it to max speed for more volume if needed, but we all have digital amps to crank it up if needed.
// equation: (LPI)*(25.4 mm per inch)/(60 seconds per minute)/(cutting RPM)=(lateral speed of cutter head)
const unsigned int FORTY_FIVE_SPEED = 217;    // 0 to 255
const unsigned int THIRTY_THREE_SPEED = 217;  // 0 to 255
const unsigned int TRIM_SPEED = 200;          // 0 to 255, trimming is 8x faster than cutting because the step size is different

// The above speeds and distances are for cutting. Lead in and lead out are specified differently.
// Speeds listed here will use DOUBLE, or SINGLE stepping, rather than microstep.

// use DOUBLE step
// const unsigned char FORTY_FIVE_LEADIN_SPEED = 81; // cutting speed is 0.5mm/s, leadin is ~1.5 mm/s
// const unsigned char THIRTY_THREE_LEADIN_SPEED = 81;

// const unsigned char FORTY_FIVE_RUNOUT_SPEED;
// const unsigned char THIRTY_THREE_RUNOUT_SPEED;


enum state_t {
  IDLE,
  TRIM_FWD,
  TRIM_BAK,
  RUN_45,
  RUN_33,
  STOP_45,
  STOP_33,
  END_45,
  END_33,
  REWIND
};

String stateName(state_t state) {
  switch (state) {
    case 0: return "IDLE"; break;
    case 1: return "TRIM_FWD"; break;
    case 2: return "TRIM_BAK"; break;
    case 3: return "RUN_45"; break;
    case 4: return "RUN_33"; break;
    case 5: return "STOP_45"; break;
    case 6: return "STOP_33"; break;
    case 7: return "END_45"; break;
    case 8: return "END_33"; break;
    case 9: return "REWIND"; break;
    default: return "IDLE"; break;
  }
}


state_t state = IDLE;
int targetDistance = 0;
int speed = 0;
// int leadInDistance = 0; //
// int runoutDistance = 0; //

Adafruit_MotorShield AFMS(0x60);  // Default address, no jumpers
Servo cutterHead;                 // create servo object to control a servo
// angle to lower the cutter head to
// finetuning is not done here, since only degree-precision. Use the needle setscrew to adjust.
const unsigned int LOWER = 180;
const unsigned int RAISE = 0;

// Connect stepper with 200 steps per revolution (1.8 degree), to motor spot 1 (M1+M2 on the board  )
Adafruit_StepperMotor *stepperPtr = AFMS.getStepper(200, 1);

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// // wrappers for the motor!
void forwardstep() {
  stepperPtr->onestep(FORWARD, MICROSTEP);
}
void backwardstep() {
  stepperPtr->onestep(BACKWARD, MICROSTEP);
}

// double step is 8x faster than microstep, lose some precision
void forwardstepDOUBLE() {
  stepperPtr->onestep(FORWARD, DOUBLE);
}
void backwardstepDOUBLE() {
  stepperPtr->onestep(BACKWARD, DOUBLE);
}

// wrap the stepper in an AccelStepper object
AccelStepper motor(forwardstep, backwardstep);
// AccelStepper doubleMotor(forwardstepDOUBLE, backwardstepDOUBLE);

void STOP_ISR() {
  if (state == RUN_45) {
    motor.stop();  // Stop the stepper motor
                   // TODO: raise the cutter head
    state = STOP_45;

  } else if (state == RUN_33) {
    motor.stop();  // Stop the stepper motor
                   // TODO: raise the cutter head
    state = STOP_33;
  } else state = state;
  while (digitalRead(KEY_STOP_REW) == LOW)
    ;  // wait until the button is released (goes back high) before exiting ISR
}

void setup() {
  Serial.begin(115200);
  // while (!Serial) {
  //   ;  // wait for serial port to connect. Needed for native USB
  // }

  noInterrupts();

  // // configure pins (all active low)
  pinMode(KEY_RUN, INPUT_PULLUP);
  pinMode(KEY_STOP_REW, INPUT_PULLUP);
  pinMode(KEY_TRIM_FWD, INPUT_PULLUP);
  pinMode(KEY_TRIM_BAK, INPUT_PULLUP);
  pinMode(SW_MODE, INPUT_PULLUP);
  pinMode(SERVO_PIN, OUTPUT);
  AFMS.begin();  // Start the shield
  // changing the i2c speed lets you microstep 4x faster
  // TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz
  // attach the interrupt to the 'emergency stop' button, falling edge since active low
  attachInterrupt(digitalPinToInterrupt(KEY_STOP_REW), STOP_ISR, FALLING);
  // TODO: can get rid of these once it's all being set in states
  // motor.setMaxSpeed(100.0);
  motor.setAcceleration(200.0);
  cutterHead.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  interrupts();
}
int loopsRun = 0;
int startPosition = 0;  // STOP always returns to 0 (position at boot), unless you trimmed, then it will adjust to the trimmed location.
void printStateSpeedDistance() {
  Serial.print("State: ");
  Serial.print(stateName(state));
  Serial.print(", speed: ");
  Serial.print(speed);
  Serial.print(", distance: ");
  Serial.println(targetDistance);
}
void loop() {
  switch (state) {
    case IDLE:
      motor.stop();
      // press RUN
      if (digitalRead(KEY_RUN) == LOW) {
        Serial.println("run");
        delay(200);
        // mode switch low->45, high->33
        if (digitalRead(SW_MODE) == LOW) {
          // run_45
          targetDistance = FORTY_FIVE_DISTANCE;
          speed = FORTY_FIVE_SPEED;
          state = RUN_45;
          motor.setSpeed(speed);
          printStateSpeedDistance();
          delay(200);
          motor.moveTo(targetDistance);
        } else {
          // run_33
          targetDistance = THIRTY_THREE_DISTANCE;
          speed = THIRTY_THREE_SPEED;
          state = RUN_33;
          motor.setSpeed(speed);
          motor.moveTo(targetDistance);
          printStateSpeedDistance();
        }
      } else if (digitalRead(KEY_TRIM_FWD) == LOW) {
        state = TRIM_FWD;
        printStateSpeedDistance();
        delay(200);
      } else if (digitalRead(KEY_TRIM_BAK) == LOW) {
        state = TRIM_BAK;
        printStateSpeedDistance();
      } else {
        state = IDLE;
        // printStateSpeedDistance();
        delay(200);
      };
      break;
    case TRIM_FWD:
      if (digitalRead(KEY_TRIM_FWD) == HIGH) {
        state = IDLE;
      } else {
        state = TRIM_FWD;
        // doubleMotor.setSpeed(TRIM_SPEED);
        // doubleMotor.runSpeed();
        // startPosition = doubleMotor.currentPosition();
        Serial.print("Current Position:");
        Serial.println(startPosition);
      }
      break;
    case TRIM_BAK:
      if (digitalRead(KEY_TRIM_BAK) == HIGH) {
        state = IDLE;
      } else {
        state = TRIM_BAK;
        // doubleMotor.setSpeed(-TRIM_SPEED);
        // doubleMotor.runSpeed();
        // startPosition = doubleMotor.currentPosition();
        Serial.print("Current Position:");
        Serial.println(startPosition);
      }
      break;
    case RUN_45:
      if (digitalRead(KEY_STOP_REW) == LOW) {
        state = STOP_45;
        Serial.println("Stopping 45");
      } else if (motor.distanceToGo() == 0) {
        // don't need to call motor.stop() here, distanceToGo handles it.
        // raise the cutter head
        cutterHead.write(RAISE);
        state = END_45;
        Serial.println("Ending 45");
      } else {
        // lower the cutter head
        cutterHead.write(LOWER);
        motor.run();
        state = RUN_45;
      }
      break;
    case RUN_33:
      if (digitalRead(KEY_STOP_REW) == LOW) {
        state = STOP_33;
        Serial.println("Stopping 33");
      } else if (motor.distanceToGo() == 0) {
        // don't need to call motor.stop() here, distanceToGo handles it.
        // raise the cutter head
        cutterHead.write(RAISE);
        state = END_33;
        Serial.println("Ending 33");
      } else {
        // lower the cutter head
        cutterHead.write(LOWER);
        motor.run();
        state = RUN_33;
      }
      break;
    case END_45:
      // rewind the motor
      motor.moveTo(0);  // TODO: constants (-distance, speed (doesnt matter for REW), acceleration)
      if (motor.distanceToGo() == 0) {
        state = IDLE;
        Serial.println("Idle, finished 45");
      } else {
        motor.run();
        state = END_45;
      }
      break;
    case END_33:
      // rewind the motor
      motor.moveTo(0);  // TODO: constants (-distance, speed (doesnt matter for REW), acceleration)
      if (motor.distanceToGo() == 0) {
        state = IDLE;
        Serial.println("Idle, finished 33");
      } else {
        motor.run();
        state = END_33;
      }
      break;
    case STOP_45:
      // raise the cutter head
      if (digitalRead(KEY_RUN) == LOW) {  // active low, wait in the ISR until its released
        state = RUN_45;                   // resume (for testing)
        Serial.println("Resuming 45");
      } else if (digitalRead(KEY_STOP_REW) == LOW) {
        state = REWIND;
      } else {
        state = STOP_45;
      }
      break;
    case STOP_33:
      // raise the cutter head
      if (digitalRead(KEY_RUN) == LOW) {  // active low, wait in the ISR until its released
        state = RUN_45;                   // resume (for testing)
        Serial.println("Resuming 33");
      } else if (digitalRead(KEY_STOP_REW) == LOW) {
        state = REWIND;
      } else {
        state = STOP_33;
      }
      break;
    case REWIND:
      // rewind the motor
      // motor.moveTo(-(targetDistance-motor.distanceToGo())); // go back however far we came
      if (motor.distanceToGo() == 0) {
        state = IDLE;
        Serial.println("Idle, finished rewind");
      } else {
        motor.run();
        state = REWIND;
      }
      break;
    default:
      state = IDLE;
      break;
  }
  // Change direction at the limits
}
