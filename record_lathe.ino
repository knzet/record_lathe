// record lathe firmware
// built for arduino uno, Adafruit Motorshield v2 library
// 
// Author: kenzie

#include <Adafruit_MotorShield.h> // https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
#include <AccelStepper.h> // https://github.com/adafruit/AccelStepper

// Linear Actuator hardware:
// https://github.com/uStepper/Linear_Actuator

// cutter head control
#include <Servo.h>

#define KEY_RUN 6
#define KEY_STOP_REW 2
#define KEY_TRIM_FWD 3
#define KEY_TRIM_BAK 4
#define SW_MODE 5
#define SERVO_PIN 10

// defined by the radius of record, to automatically stop.
// in a later version, the music file ending playback should trigger the spiral-out. A limit switch, either optical or momentary, would be the simplest option for a hard stop.
#define FORTY_FIVE_DISTANCE 20    //1000
#define THIRTY_THREE_DISTANCE 20  //2700*8

// cutting pitch
// complete silence is 555 LPI (very slow pitch -> lines very close together, no room for amplitude)
// 0dB for a 33 is 200 LPI (faster than current setup can microstep)
// variable groove pitch is too fancy for now, since the audio is played through computer.
// potential future version could hook an opamp off the audio stream, measure rms level and speed up or slow down to give more space. Would really need finetuning.
// compromise is to cut about as fast as we can on this hardware, while keeping the smoothness of microstepping
// 217/255 is 0.52 mm/s, which corresponds to 224 LPI at 45 RPM. This RPM value is CUTTING, NOT PLAYBACK. Could push it to max speed for more volume if needed, but we all have digital amps to crank it up if needed.
// equation: (LPI)*(25.4 mm per inch)/(60 seconds per minute)/(cutting RPM)=(lateral speed of cutter head)
#define FORTY_FIVE_SPEED 217    // 0 to 255
#define THIRTY_THREE_SPEED 217  // 0 to 255
#define TRIM_SPEED 200          // 0 to 255, 

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

state_t state = IDLE;
int targetDistance = 0;
int speed = 0;

Adafruit_MotorShield AFMS(0x60);  // Default address, no jumpers
Servo cutterHead;                 // create servo object to control a servo
#define LOWER 180
#define RAISE 0

// Connect stepper with 200 steps per revolution (1.8 degree), to motor spot 1 (M1+M2 on the board)
Adafruit_StepperMotor *stepperPtr = AFMS.getStepper(200, 1);

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// // wrappers for the motor!
void forwardstep() {
  stepperPtr->onestep(FORWARD, MICROSTEP);
}
void backwardstep() {
  stepperPtr->onestep(BACKWARD, MICROSTEP);
}

void forwardstepTRIM() {
  stepperPtr->onestep(FORWARD, DOUBLE);
}
void backwardstepTRIM() {
  stepperPtr->onestep(BACKWARD, DOUBLE);
}

// wrap the stepper in an AccelStepper object
AccelStepper motor(forwardstep, backwardstep);
AccelStepper trim(forwardstepTRIM, backwardstepTRIM);

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
  // // attach the interrupt to the 'emergency stop' button, falling edge since active low
  attachInterrupt(digitalPinToInterrupt(KEY_STOP_REW), STOP_ISR, FALLING);
  // TODO: can get rid of these once it's all being set in states
  // motor.setMaxSpeed(100.0);
  motor.setAcceleration(200.0);
  cutterHead.attach(SERVO_PIN);  // attaches the servo on pin 10 to the servo object
  interrupts();
}
int loopsRun = 0;
void printStateSpeedDistance() {
  Serial.print("State: ");
  Serial.print(state);
  Serial.print(", speed: ");
  Serial.print(speed);
  Serial.print(", distance: ");
  Serial.println(targetDistance);
}
void loop() {
  // delay(200);
  // Serial.println("IDLE");
  switch (state) {
    case IDLE:
      motor.stop();
      // press RUN
      if (digitalRead(KEY_RUN) == LOW) {
        // mode switch low->45, high->33
        if (digitalRead(SW_MODE) == LOW) {
          // run_45
          targetDistance = FORTY_FIVE_DISTANCE;
          speed = FORTY_FIVE_SPEED;
          state = RUN_45;
          motor.setSpeed(speed);
          motor.moveTo(targetDistance);
          printStateSpeedDistance();
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
      } else if (digitalRead(KEY_TRIM_BAK) == LOW) {
        state = TRIM_BAK;
        printStateSpeedDistance();
      } else {
        state = IDLE;
        printStateSpeedDistance();
        delay(200);
      };
      break;
    case TRIM_FWD:
      if (digitalRead(KEY_TRIM_FWD) == HIGH) {
        state = IDLE;
      } else {
        state = TRIM_FWD;
        trim.setSpeed(TRIM_SPEED);
        trim.runSpeed();
      }
      break;
    case TRIM_BAK:
      if (digitalRead(KEY_TRIM_BAK) == HIGH) {
        state = IDLE;
      } else {
        state = TRIM_BAK;
        trim.setSpeed(-TRIM_SPEED);
        trim.runSpeed();
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
