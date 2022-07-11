#include <Wire.h>
#include <LSM6.h>

#define I2C_ADDR  8

#define FWD LOW
#define REV HIGH

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

unsigned long update_ts;
#define UPDATE_MS 100

// Data to send(tx) and receive(rx)
// on the i2c bus.
// Needs to match the master device
// Note that the Arduino is limited to
// a buffer of 32 bytes!
#pragma pack(1)
typedef struct i2c_status {
  float x;                  // 4 bytes
  float y;                  // 4 bytes
  float theta;              // 4 bytes
  int8_t status;            // 1 byte
} i2c_status_t;
#pragma pack()

i2c_status_t i2c_status_tx;
volatile i2c_status_t i2c_status_rx;

LSM6 imu;

void setLeftMotor(int velocity) {
  if (velocity >= 0) {
    digitalWrite(L_DIR_PIN, FWD);
  }
  else {
    digitalWrite(L_DIR_PIN, REV);
  }

  analogWrite(L_PWM_PIN, abs(velocity));
}

void setRightMotor(int velocity) {
  if (velocity >= 0) {
    digitalWrite(R_DIR_PIN, FWD);
  }
  else {
    digitalWrite(R_DIR_PIN, REV);
  }

  analogWrite(R_PWM_PIN, abs(velocity));
}

// put your setup code here, to run once:
void setup() {
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  setLeftMotor(50);
  setRightMotor(0);

  // Serial for debugging.
  Serial.begin(9600);
  Serial.println("***RESTART***");
  delay(1000);

  // Clear out i2c data structs
  memset( (void*)&i2c_status_tx, 0, sizeof( i2c_status_tx ) );
  memset( (void*)&i2c_status_rx, 0, sizeof( i2c_status_rx ) );

  // Begin I2C as a slave device.
  Wire.begin( I2C_ADDR );
  Wire.onRequest( i2c_sendStatus );
  Wire.onReceive( i2c_recvStatus );

  if (!imu.init() ) {
    // Since we failed to communicate with the
    // IMU, we put the robot into an infinite
    // while loop and report the error.
    // while (1) {
    //   Serial.println("Failed to detect and initialize IMU!");
    //   delay(1000);
    // }
  }

  // IMU initialise ok!
  // Set the IMU with default settings.
  // imu.enableDefault();

}

// put your main code here, to run repeatedly:
void loop() {

  // General update, here updating not PID controllers
  // if ( millis() - update_ts > UPDATE_MS ) {
  //   update_ts = millis();

  //   Serial.println("Rx Status:");
  //   printRXStatus();
  //   Serial.println("\n");
  // }

  //Serial.println( "loop" );
  delay(100);
}

// When the Core2 calls an i2c request, this function
// is executed.  Sends robot status to Core2.
void i2c_sendStatus() {

  // Populate our current status
  i2c_status_tx.x = 123.456;
  i2c_status_tx.y = 789.1011;
  i2c_status_tx.theta = 12.13;
  i2c_status_tx.status--; // debugging

  // Send up
  Wire.write( (byte*)&i2c_status_tx, sizeof( i2c_status_tx ) );

}

// When the Core2 calls and i2c write, the robot
// will call this function to receive the data down.
void i2c_recvStatus(int len ) {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW); 


  Wire.readBytes( (byte*)&i2c_status_rx, sizeof( i2c_status_rx ) );

  setLeftMotor(i2c_status_rx.x);
  setRightMotor(i2c_status_rx.x);

  /*
    // I've had a lot of trouble with this bit of code.
    // Arduino wire (i2c) has a hard limit of 32 bytes
    // buffer.
    uint8_t buf[ 32 ];
    int i = 0;
    while( Wire.available() && i < 32) {
    buf[i] = Wire.read();
    i++;
    }
    memcpy( &i2c_status_rx, buf, sizeof( i2c_status_rx) );
  */
}


void printRXStatus() {

  Serial.println(i2c_status_rx.x);
  Serial.println(i2c_status_rx.y);
  Serial.println(i2c_status_rx.theta);
  Serial.println(i2c_status_rx.status);
}
