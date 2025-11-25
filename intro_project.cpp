#include <arduino_freertos.h>
#include <stdint.h>

/*
The car has three possible functional states:
1. Low Voltage (LV)
2. Tractive System (TS)
3. Ready to Drive (RTD)

When in LV, the car cannot drive. When in TS, the car is energized, but still
cannot drive. When in RTD, the car is energized and able to drive.

There are two Accumulator Isolation Relays (AIRs) that connect the high voltage
(HV) battery to the rest of the car. One is on the positive terminal, and the
other on the negative terminal (AIR+ and AIR-). That is to say, the battery is
disconnected unless we energize both relays, which is important for safety.
Whenever we have a critical error, we can open these relays / de-energize them
to make sure there is no HV outside the accumulator.

When we go from LV -> TS, we cannot simply energize the relays (ask an
electrical lead why!). Instead we first close AIR-, then another relay called
the "Precharge Relay". After ~5 seconds we close AIR+ and open the precharge
relay.

There is a TS On button and an RTD button on the dash that move the cars into
those states. ie. When you press TS On the car goes TS, and when you press RTD,
it goes to RTD. They are both active low, meaning when the button's are pressed,
the voltage goes low.

We also have a few sensors that we need to be able to calculate how much torque
to command:

1. Current sensor (How much current there is in the HV path)
2. Wheelspeeds
3. Steering Angle Sensor

The current sensor is a Hall Effect sensor read by an ADC through GPIO pins. The
conversion is: 1 amp per 10mV

Wheelspeeds are a bit tricky. The sensor is routed to a GPIO pin, and is
normally high. However, the wheels have a gear looking thing with 17 teeth --
everytime one of the teeth passes the sensor, the voltage at the GPIO pin goes
low. You can derive the RPM of the wheel from how often the voltage goes low.

The steering angle sensor comes to us via CAN.

We also have to monitor our battery to make sure nothing explodes or catches on
fire... Thankfully we have a Battery Management System (BMS) that monitors cell
voltages and temperatures and can send them to us. It sends this info via SPI.
If any cell voltage goes below 2.8V or above 4.3V, de-energize the car. Also, if
any temperature exceeds 60 degrees, de-energize.

Finally, we have an LCD on the dash which is useful to display information. You
should print all relevant info to the screen so we know what's going on with the
car. This also operates via SPI.

Mock library functions have been provided where necessary (marked with extern).

NOTE - EXTREMELY IMPORTANT: the CAN methods `can_send` and `can_receive` are NOT
thread safe.

Also think about the implications of having two peripherals on one SPI bus.

----------------------------

Requirements:

1. Command each motor with an appropriate torque every 1ms
2. Print sensor values and torque values to the LCD every 100ms
3. Monitor the highest/lowest voltage and highest temperature from the BMS and
shutdown the car if there is any unsafe condition

----------------------------

Pins:

TS On:          12
RTD:            13
Current Sensor: 19
CAN RX:         2
CAN TX:         3
MOSI:           5
MISO:           6
SCK:            7
LCD_CS:         8
BMS_CS:         9
AIR+:           22
Precharge:      23
AIR-:           10

----------------------------

"Submission" instructions. I want you all to get familiar with git, so we will
do this project with git. The repo is
https://github.com/UTFR/firmware-tutorials.

If you still haven't joined the github organization, let me know and i'll add
you. Clone the repository, and make a branch called `<your name>/intro_project`.
Copy this file into the directory `intro_projects/<your name>` and make all your
changes there.

Whenever you add a feature, `git add` the files, and `git commit -m "..."` with
a useful/descriptive message. Whenver you want your changes to be public, do
`git push origin <your name>/intro_project`.

Also, I won't enforce it for this project, but for the actual firmware repo we
have a code formatter (clang-format). It means that everyone's code will look
exactly the same, in terms of how much whitespace there is and other aesthetics
like that. It's not there for aesthetics, but moreso so that you when people
make changes you can see exactly what they changed, whereas without a formatter,
you end up with lots of useless formatting changes. You should install
clang-format as soon as possible and set it up :)
*/

// Intialize all pins

// Pin definitions(uint8_t is variable that store pin num)
static const uint8_t TS_ON      = 12;
static const uint8_t RTD        = 13;
static const uint8_t CURRENT    = 19;
static const uint8_t CAN_RX     = 2;
static const uint8_t CAN_TX     = 3;
static const uint8_t MOSI       = 5;
static const uint8_t MISO       = 6;
static const uint8_t SCK        = 7;
static const uint8_t LCD_CS     = 8;
static const uint8_t BMS_CS     = 9;
static const uint8_t AIR_PLUS   = 22;
static const uint8_t PRECHARGE  = 23;
static const uint8_t AIR_MINUS  = 10;

// car state 
static int car_state = 0;  // 0 = LV, 1 = TS, 2 = RTD

// 4 torques 

static float torques[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// say u have 12 cells
static const int NUM_CELLS = 12;  
// track maxv,minv,maxt 
static float min_cell_voltage = 0.0;
static float max_cell_voltage = 0.0;
static float max_cell_temperature = 0.0;

/*
  Initialize the CAN peripheral with given RX and TX pins at a given baudrate.
*/
extern void can_init(uint8_t rx, uint8_t tx, uint32_t baudrate);

/*
  Send a CAN message with a given id.
  The 8 byte payload is encoded as a uint64_t
*/
extern void can_send(uint8_t id, uint64_t payload);
/*
  Receive a CAN message with a given id into a uint64_t
*/
extern void can_receive(uint64_t *payload, uint8_t id);

/*
  Calculates four torques, in order, for the Front Left, Front Right, Rear Left,
  and Rear Right motors given current, wheelspeeds (in the same order), and a
  steering angle.
*/
extern void calculate_torque_cmd(
  float *torques, float current, float *wheelspeeds, float steering_angle
);

/*
  Initialize the LCD peripheral
*/
extern void lcd_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t lcs_cs);

/*
  Print something to the LCD
*/
extern void lcd_printf(const char *fmt, ...);

/*
  Initialize the BMS
*/
extern void bms_init(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t bms_cs);

/*
  Get voltage of the nth cell in the battery
*/
extern float bms_get_voltage(uint8_t n);

/*
  Get temperature of the nth cell in the battery
*/
extern float bms_get_temperature(uint8_t n);

// check bms


static bool check_bms(){
  float local_min_v = 1000.0;
  float local_max_v = 0.0;
  float local_max_t = 0.0;

  for(int i =0; i<NUM_CELLS; i++){
    float v = bms_get_voltage(i);
    float t = bms_get_temperature(i);
    if(v < local_min_v){
      local_min_v = v;
    }else if(v > local_max_v){
      local_max_v = v;
    }
    if(t > local_max_t){
      local_max_t = t;
    }

    min_cell_voltage = local_min_v;
    max_cell_voltage = local_max_v;
    max_cell_temperature = local_max_t;

    if (local_min_v < 2.8 || local_max_v > 4.3 || local_max_t > 60.0) {
    return true;
  }
  }
  return false;
}

// De-energize
static void shutdown(){
  digitalWrite(AIR_PLUS, LOW);
  digitalWrite(PRECHARGE, LOW);
  digitalWrite(AIR_MINUS, LOW);
  car_state = 0;  // back to LV

  lcd_printf("BMS unsafe\n");
  lcd_printf("State: LV\n");

}


static void torque_task(void *pvParameters) {
// Remember current RTOS time so we can iterate 1ms per loop
  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    if (car_state == 2) {
      float current = 10;
      float wheelspeeds[4] = {100, 100, 100, 100};
      float steering_angle = 0.0;

      calculate_torque_cmd(torques, current, wheelspeeds, steering_angle);
      // later: can_send(...) based on torques
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1)); // 1 ms period
  }
}



// run first and once 

void setup(void) {
  // active low 
  pinMode(TS_ON , INPUT_PULLUP);
  pinMode(RTD, INPUT_PULLUP);

  // Air-> connect the HV battery to the rest of the car that needs HV.
  pinMode(AIR_PLUS,  OUTPUT);
  pinMode(AIR_MINUS, OUTPUT);

 // Precharge Relay-> slowly charges HV 
  pinMode(PRECHARGE,OUTPUT);
   
  // Initialize them in LV state 

  digitalWrite(AIR_PLUS, LOW);
  digitalWrite(PRECHARGE, LOW);
  digitalWrite(AIR_MINUS, LOW);

  // Initialize communication with each 

  lcd_init(MOSI, MISO, SCK, LCD_CS);
  bms_init(MOSI, MISO, SCK, BMS_CS);
  // baudrate -> communication speed(bps), standard bps for can bus is 500k
  can_init(CAN_RX , CAN_TX , 500000);

  // function, name, stack size, parameter passed to the task, priority, store the task handle("ID")
  xTaskCreate(torque_task, "torque_task", 256, NULL, 2, NULL);

  lcd_printf("State: LV\n");

}

// infinite loop 

void loop(void) {
  // active low 
  bool ts_pressed  = (digitalRead(TS_ON) == LOW);
  bool rtd_pressed = (digitalRead(RTD)   == LOW);

  if(ts_pressed && car_state == 0 ){
    car_state = 1;

    digitalWrite(AIR_MINUS, HIGH);
    digitalWrite(PRECHARGE, HIGH);

    vTaskDelay(pdMS_TO_TICKS(5000));

    digitalWrite(AIR_PLUS, HIGH);

    // only need it at start 
    digitalWrite(PRECHARGE, LOW);

    lcd_printf("State: TS\n");

  }
  else if(rtd_pressed && car_state == 1){
    car_state = 2;
    lcd_printf("State: RTD\n");

  }
  if(check_bms() && car_state != 0){
    shutdown();
  }
// print

  lcd_printf(min_cell_voltage, max_cell_voltage,max_cell_temperature);


  lcd_printf(torques[0], torques[1], torques[2], torques[3]);



// so screen dont spam so quick
  vTaskDelay(pdMS_TO_TICKS(100));

}
