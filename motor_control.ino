// https://bluepad32.readthedocs.io/en/latest/plat_arduino/
// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/

#include <Bluepad32.h>
#include <Stepper.h>

// 15RPM with torque ~34.3 mN*m
const int stepsPerRevolution = 2048; // 32 steps with gear reduction of 64:1 -> 32*64=2048
const int stepSize = floor(stepsPerRevolution/360);

const int IN1 = 19; // D19
const int IN2 = 18; // D18
const int IN3 = 5;  // D5
const int IN4 = 17; // TX2

Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

// flags for throttle and brake
bool throttle_flag = false;
bool brake_flag = false;
int trigger_floor = 100;

int32_t throttle; // 0 -> 1023
int32_t brake; // 0 -> 1023


void stepperSetup() {
  myStepper.setSpeed(15);
}

// Define controller
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Function called any time new gamepad connected, up to 4 at a time.
// TODO: Look if it's reasonable to only allow one to connect, feel free to make suggestions if you may know
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index =%d\n", i);
      
      // Additionally, you can get certain gamepad properties: Model, VID, PID< BTAddr, flags, etc.
      ControllerProperties properties = ctl -> getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                    ctl->getModelName().c_str(),
                    properties.vendor_id,
                    properties.product_id
                  );
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not find empty slot.");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }
  if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void bluetoothSetup() {
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // This function should be called when user performs "device factory reset" or similar.
  // Forgetting BT keys prevents "paired" gamepads to reconnect and might also fix some connection issues.
  BP32.forgetBluetoothKeys();

  // Disable virtual device (used for controllers like DualSense and DualShock4) by default
  BP32.enableVirtualDevice(false);
}

// The main setup function for the microcontroller
void setup() {
  Serial.begin(115200);

  stepperSetup();

  bluetoothSetup();
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
  throttle = ctl -> throttle();
  brake = ctl -> brake();
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController -> isConnected() && myController -> hasData()) {
      if (myController -> isGamepad()) processGamepad(myController);
      else Serial.println("Unsupported controller");
    }
  }
}

void autoStep() {
  myStepper.step(stepsPerRevolution);
  delay(500);

  myStepper.step(-stepsPerRevolution);
  delay(500);
}

void driveAllLow(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
   digitalWrite(p1,LOW);
   digitalWrite(p2,LOW);
   digitalWrite(p3,LOW);
   digitalWrite(p4,LOW);
}

void drive() {
  throttle_flag = throttle > trigger_floor;
  brake_flag = brake > trigger_floor;

  if (throttle_flag && brake_flag) {
    driveAllLow(IN1, IN2, IN3, IN4);
  } else {
    if (throttle_flag) myStepper.step(stepSize);
    if (brake_flag) myStepper.step(-stepSize);
  }
}

void loop() {
  drive();
  
  bool dataUpdated = BP32.update();

  if (dataUpdated) processControllers();
}
