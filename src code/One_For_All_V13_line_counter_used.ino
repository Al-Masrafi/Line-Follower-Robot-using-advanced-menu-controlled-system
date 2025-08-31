#include <U8glib.h>
#include <Pushbutton.h>
#define MASK2_2 B01100110  // 0x66
#define MASK2_0 B01100000  // 0x60
#define MASK0_2 B00000110  // 0x06
#define MASK3_3 B11100111  // 0xe7
#define MASK0_3 B00000111  // 0x07
#define MASK3_0 B11100000  // 0xe0
#define MASK4_0 B11110000  // 0xf0
#define MASK0_4 B00001111  // 0x0f
#define MASK4_4 B11111111  // 0xff


// Counter for the number of cross sections detected
int crossSectionCount = 0;

// Maximum number of cross sections before stopping (set as needed)
const int MAX_CROSS_SECTIONS = 10;

// Flag to indicate if the robot should stop
bool stopRobot = false;

// Flag to indicate that an intersection has been detected and is being processed
bool intersectionDetected = false;

// Time when the last intersection was detected
unsigned long lastIntersectionTime = 0;

// Minimum time between intersection detections (in milliseconds)
const unsigned long INTERSECTION_COOLDOWN = 1000;  // 1 second

/***********************************************************************
  /* Cross line detection processing
  /* Return values: 0: no cross line, 1: cross line
  /***********************************************************************/
// Reference variable for the line sensor in order to determine the white and black lines or colors
int HighRef[8];
int LowRef[8];
int ResultRef[8];
int threshold[10];
byte sensorPin[8] = { 3, 1, 0, 2, 4, 6, 8, 10 };
boolean digital_value[10];
int SensorCount = 8;
int dcount = 0;
//PID values
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 120;
int MAXSPEED = 200;  //150
const int MINSPEED = 0;
// Add these constants at the top
#define PRE_TURN_DISTANCE 150        // Move forward 15cm (adjust based on robot speed)
#define TURN_SPEED 100               // Turning speed (0-255)
#define LINE_REACQUIRE_TIMEOUT 3000  // Max time to search for line

float Kp = 0.001;  //2.0, 0.005bestttt, 0.05 also best
float Kd = 5;   //5.0,8.0,1.0 best,1.2,1.5,
float Ki = 0.0;

// Variable for Menu or Mode
bool calibration_mode = false;
bool sensor_check_mode = false;
bool start_mode = false;
bool follow_line = false;
int calibration_process = 1;  // Variable for the sequence of the calibration process

// Define the pin numbers for each button
const int BUTTON_PIN_1 = 31;
const int BUTTON_PIN_2 = 29;
const int BUTTON_PIN_3 = 27;

// Create instances of PushButton for each button
Pushbutton button1(BUTTON_PIN_1);
Pushbutton button2(BUTTON_PIN_2);
Pushbutton button3(BUTTON_PIN_3);

// Define the OLED display object
U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);

#define MENU_ITEMS 4
char *menu_strings[MENU_ITEMS] = {
  "Calibration",
  "Sensor Test",
  "Follow Line",
  "RUN"
};
uint8_t menu_current = 0;          // Current selected menu item
uint8_t menu_redraw_required = 1;  // Flag to indicate if menu needs to be redrawn

#define MAX_SUBMENU_ITEMS 5
char *submenu_strings1[MAX_SUBMENU_ITEMS] = {
  "MOTOR OFF",
  "LFSPEED",
  "MAXSPEED",
  "BACK"
};
uint8_t submenu_count1 = 4;
uint8_t submenu_current1 = 0;
// Navigation Menu
int MENU_NO = 1;
// Additional state for motor status
bool motor_on = false;
// Flag for adjusting mode
bool adjusting_lfspeed = false;
bool adjusting_maxspeed = false;
// Motor pins declaration
const int IN1 = 46;  // Control pin 1 for Motor A
const int IN2 = 47;  // Control pin 2 for Motor A
const int IN3 = 50;  // Control pin 1 for Motor B
const int IN4 = 52;  // Control pin 2 for Motor B
const int STD = 48;  // Standby pin for motor driver
const int ENA = 44;  // PWM pin for Motor A speed control
const int ENB = 45;  // PWM pin for Motor B speed control

// Timing variables for Serial printing
unsigned long previousMillis = 0;   // Stores the last time data was printed
const float interval_limit = 10.0;  // Interval limit for printing data (in seconds)
float interval_elapsed = 0;         // Elapsed interval time
int pattern;

// Add this enum definition BEFORE any functions that use it
enum JunctionType {
  NONE,
  SHARP_TURN,
  T_JUNCTION,
  Y_JUNCTION
};

// Add navigation states enum
enum NavigationState {
  FOLLOWING_LINE,
  PRE_TURN,
  TURNING,
  LINE_SEARCH
};

// Add these global variables AFTER enum definitions
NavigationState navState = FOLLOWING_LINE;
unsigned long stateStartTime = 0;
int turnDirection = 0;  // 1=left, -1=right

// Function prototypes (add these BEFORE setup())
JunctionType detect_junction();
void handle_junction_detection(JunctionType detected);



void setup() {
  // Initialize the OLED display
  u8g.begin();
  // Serial.begin(115200);
  // Serial.println("CLEARSHEET");
  // Serial.println("CLEARDATA");

  // Initialize motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(STD, HIGH);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Setup button pins as inputs with internal pull-up resistors
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  pinMode(BUTTON_PIN_3, INPUT_PULLUP);
}

void loop() {
  // unsigned long currentMillis = millis();
  if (menu_redraw_required != 0) {
    u8g.firstPage();  // Begin first page
    do {
      switch (MENU_NO) {
        case 1:
          draw_menu();  // Draw main menu items
          break;
        case 2:
          draw_submenu1(menu_current);  // Draw current submenu1 items
          break;
        default:
          draw_menu();  // Default to main menu if MENU_NO is invalid
          break;
      }
    } while (u8g.nextPage());  // Continue to next page if available
    menu_redraw_required = 0;  // Menu updated, reset redraw flag
  }
  update_menu();  // Check for menu navigation

  // Conditions to start the line sensor calibration process
  if (calibration_mode == true) {
    calibration();  // Go to the calibration subroutine
  }
  // Conditions for checking line sensors and displaying on OLED Display
  if (sensor_check_mode == true) {
    sensor_check();
  }
  /////////////////////////////////////////////////////////////////////////
  if (start_mode == true) {
    line_follow5();
  }
  // Conditions for stopping and starting the motor
  if (motor_on == true) {
    motor_control(255, 255);
  } else if (motor_on == false) {
    motor_control(0, 0);
  }
  if (adjusting_lfspeed == true) {
    while (adjusting_lfspeed) {
      bool up_inner = button1.isPressed();
      bool down_inner = button3.isPressed();
      bool enter_inner = button2.isPressed();

      if (up_inner) {
        lfspeed = min(lfspeed + 5, MAXSPEED);  // Increase lfspeed, up to MAXSPEED
        menu_redraw_required = 1;
        delay(200);
      } else if (down_inner) {
        lfspeed = max(lfspeed - 5, MINSPEED);  // Decrease lfspeed, down to MINSPEED
        menu_redraw_required = 1;
        delay(200);
      } else if (button2.isPressed()) {
        adjusting_lfspeed = false;
        while (button2.isPressed()) {
          delay(100);
        }
        menu_redraw_required = 1;
        delay(10);
      }
      u8g.firstPage();
      do {
        u8g.setPrintPos(16, 8);
        u8g.print("LFSPEED");
        u8g.setPrintPos(16, 16);
        u8g.print(lfspeed);
      } while (u8g.nextPage());
    }
  }
  if (adjusting_maxspeed == true) {
    while (adjusting_maxspeed) {
      bool up_inner = button1.isPressed();
      bool down_inner = button3.isPressed();
      bool enter_inner = button2.isPressed();

      if (up_inner) {
        MAXSPEED = min(MAXSPEED + 5, 255);  // Increase lfspeed, up to MAXSPEED
        menu_redraw_required = 1;
        delay(200);
      } else if (down_inner) {
        MAXSPEED = max(MAXSPEED - 5, lfspeed);  // Decrease lfspeed, down to MINSPEED
        menu_redraw_required = 1;
        delay(200);
      } else if (button2.isPressed()) {
        adjusting_maxspeed = false;
        while (button2.isPressed()) {
          delay(100);
        }
        menu_redraw_required = 1;
        delay(10);
      }
      u8g.firstPage();
      do {
        u8g.setPrintPos(16, 8);
        u8g.print("MAXSPEED");
        u8g.setPrintPos(16, 16);
        u8g.print(MAXSPEED);
      } while (u8g.nextPage());
    }
  }
  //detectintersection();
}

/////////////////////////////////////////////////////////////

void display_stop_message() {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_5x7);
    u8g.setPrintPos(20, 16);
    u8g.print("Robot Stopped");
    u8g.setPrintPos(20, 24);
    u8g.print("Intersection reached");
  } while (u8g.nextPage());
  delay(2000);  // Display message for 2 seconds
}


void draw_menu() {
  u8g.setFont(u8g_font_5x7);  // Set font size
  for (uint8_t i = 0; i < MENU_ITEMS; i++) {
    u8g.setPrintPos(0, 8 + (i * 8));
    if (i == menu_current) {
      u8g.print("> ");
    } else {
      u8g.print("  ");
    }
    u8g.print(menu_strings[i]);
  }
}


void draw_submenu1(uint8_t menu) {
  u8g.setFont(u8g_font_5x7);  // Set font size
  for (uint8_t i = 0; i < submenu_count1; i++) {
    u8g.setPrintPos(0, 8 + (i * 8));
    if (i == submenu_current1) {
      u8g.print("> ");
    } else {
      u8g.print("  ");
    }

    if (i == 0) {
      // Display motor status
      u8g.print(motor_on ? "MOTOR ON" : "MOTOR OFF");
    } else if (i == 1) {
      // Display lfspeed option with current value
      u8g.print(submenu_strings1[i]);
      u8g.print(": ");
      u8g.print(lfspeed);
      // u8g.setPrintPos(80, 16);
      // u8g.print(adjusting_lfspeed ? "< " : "  ");
    } else if (i == 2) {
      u8g.print(submenu_strings1[i]);
      u8g.print(": ");
      u8g.print(MAXSPEED);
    } else {
      u8g.print(submenu_strings1[i]);
    }
  }
}

void update_menu() {
  // Read button states (note: using INPUT_PULLUP reverses logic, so LOW indicates button press)
  bool up = button3.isPressed();     // Button press is LOW due to INPUT_PULLUP
  bool down = button1.isPressed();   // Button press is LOW due to INPUT_PULLUP
  bool enter = button2.isPressed();  // Button press is LOW due to INPUT_PULLUP

  if (MENU_NO == 1) {
    if (up) {
      menu_current++;
      if (menu_current >= MENU_ITEMS) {
        menu_current = 0;
      }
      menu_redraw_required = 1;  // Redraw menu after navigation
      delay(200);                // Optional debounce delay
    } else if (down) {
      if (menu_current == 0) {
        menu_current = MENU_ITEMS - 1;
      } else {
        menu_current--;
      }
      menu_redraw_required = 1;  // Redraw menu after navigation
      delay(200);                // Optional debounce delay
    } else if (enter) {
      if (menu_current == 3) {
        MENU_NO++;             // Enter submenu or update menu dimensions
        submenu_current1 = 0;  // Reset submenu1 selection
      } else if (menu_current == 2) {
        start_mode = !start_mode;
        // Reset navigation state when starting
        if (start_mode) {
          navState = FOLLOWING_LINE;
          stopRobot = false;
          //crossSectionCount = 0;
        }
      } else if (menu_current == 1) {
        sensor_check_mode = true;
        // while (enter) {
        //   delay(100);
        // }
      } else if (menu_current == 0) {
        calibration_mode = true;
        // while (enter) {
        //   delay(100);
        // }
      }
      menu_redraw_required = 1;  // Redraw menu after entering submenu1
      delay(200);                // Optional debounce delay
    }
  } else if (MENU_NO == 2) {  // Inside submenu1
    if (up) {
      submenu_current1++;
      if (submenu_current1 >= submenu_count1) {
        submenu_current1 = 0;
      }
      menu_redraw_required = 1;  // Redraw menu after navigation
      delay(200);                // Optional debounce delay
    } else if (down) {
      if (submenu_current1 == 0) {
        submenu_current1 = submenu_count1 - 1;
      } else {
        submenu_current1--;
      }
      menu_redraw_required = 1;  // Redraw menu after navigation
      delay(200);                // Optional debounce delay
    } else if (enter) {
      // Enter submenu2 or process submenu1 selection
      if (submenu_current1 == submenu_count1 - 1) {
        // Back option selected, exit submenu1
        MENU_NO = 1;               // Return to main menu
        menu_redraw_required = 1;  // Redraw menu after exiting submenu1
        delay(200);                // Optional debounce delay
      } else if (submenu_current1 == 0) {
        // Toggle motor state
        motor_on = !motor_on;
        menu_redraw_required = 1;  // Redraw menu to show updated motor state
        delay(200);                // Optional debounce delay
      } else if (submenu_current1 == 1) {
        // Change lfspeed value
        adjusting_lfspeed = true;  // Add a flag for adjusting mode
        menu_redraw_required = 1;  // Redraw menu to show updated motor state
        delay(200);

        // if (menu_redraw_required) {
        //   u8g.firstPage();
        //   do {
        //     draw_submenu1(menu_current);
        //   } while (u8g.nextPage());
        //   menu_redraw_required = false; // Reset flag
        // }
        //Update display with new lfspeed value
        // u8g.firstPage();
        // do {
        //   draw_submenu1(menu_current);
        //   //   if (adjusting_lfspeed) {
        //   //   u8g.setFont(u8g_font_6x10);
        //   //   u8g.drawStr(80, 16, ">");  // Draw arrow symbol at desired position
        //   // }
        // } while (u8g.nextPage());

        // delay(10);  // Prevent rapid cycling
      } else if (submenu_current1 == 2) {
        adjusting_maxspeed = true;
        menu_redraw_required = 1;  // Redraw menu to show updated motor state
        delay(200);
      }
    }
  }
}

void motor_control(int leftSpeed, int rightSpeed) {
  // Ensure speed values are within the valid range (-255 to 255)
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Set motor direction pins for Motor A
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);  // Motor A forward
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);  // Motor A reverse
    digitalWrite(IN2, HIGH);
  }

  // Set motor direction pins for Motor B
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);  // Motor B forward
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);  // Motor B reverse
    digitalWrite(IN4, HIGH);
  }

  // Set motor speeds using PWM (0 to 255)
  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));
}
///////////////////////////////////////////////////////////////////////////////////////////////
void calibration() {
  // Configuration and initial display of the calibration process
  calibration_process = 1;


  for (byte i = 0; i < 8; i++) {
    HighRef[i] = 0;
    LowRef[i] = 1023;
  }

  while (calibration_mode == true) {
    if (button2.isPressed()) {
      calibration_process++;
      while (button2.isPressed()) {
        delay(100);
      }
    }

    // Conditions to get the highest and lowest reference value of the line
    if (calibration_process == 1) {
      for (byte i = 0; i < 8; i++) {
        int sensorValue = analogRead(sensorPin[i]);
        if (sensorValue > HighRef[i]) HighRef[i] = sensorValue;
        if (sensorValue < LowRef[i]) LowRef[i] = sensorValue;
      }
      u8g.firstPage();
      do {
        u8g.setPrintPos(20, 8);
        u8g.print("CALIBRATION....");
        for (byte i = 0; i < 8; i++) {
          u8g.setPrintPos(0 + i * 16, 16);
          u8g.print(HighRef[i]);
          u8g.setPrintPos(0 + i * 16, 24);
          u8g.print(LowRef[i]);
        }
        u8g.setPrintPos(0, 32);
        u8g.print("Press ENTER(Done)");

      } while (u8g.nextPage());
    }




    // Conditions for obtaining the reference result value
    if (calibration_process == 2) {
      for (byte i = 0; i < 8; i++) {
        ResultRef[i] = HighRef[i] + LowRef[i];
        ResultRef[i] = ResultRef[i] / 2;
      }

      u8g.firstPage();
      do {
        u8g.setPrintPos(20, 8);
        u8g.print("CALIBRATION");
        u8g.setPrintPos(20, 16);
        u8g.print("IS COMPLETE");
      } while (u8g.nextPage());

      delay(1000);

      calibration_process = 3;
    }

    // Conditions for checking sensors based on the results of the calibration
    if (calibration_process == 3) {
      readSensor();
      u8g.firstPage();
      do {
        u8g.setPrintPos(16, 14);
        u8g.print("CHECK SENSOR");
        u8g.setPrintPos(33, 29);
        printBinaryByte(readSensor());
        // u8g.setPrintPos(0, 59);
        // u8g.print("1. BACK");
      } while (u8g.nextPage());
      delay(10);
    }

    // Conditions for returning to the void loop (calibration process is complete)
    if (calibration_process == 4) {
      calibration_mode = false;

      return;
    }
  }
}
void sensor_check() {
  while (sensor_check_mode == true) {
    readSensor();
    PID2();
    u8g.firstPage();
    do {
      u8g.setPrintPos(16, 8);
      u8g.print("CHECK SENSOR");
      u8g.setPrintPos(33, 16);
      printBinaryByte(readSensor());
      // printBinaryByte(sensor_inp(MASK3_3));
      u8g.setPrintPos(0, 24);
      u8g.print("Position= ");
      u8g.setPrintPos(50, 24);
      u8g.print(position());
      u8g.setPrintPos(0, 32);
      u8g.print(lsp);
      u8g.setPrintPos(50, 32);
      u8g.print(rsp);


      // u8g.setPrintPos(0, 32);
      // u8g.print("Press Enter For Back");
    } while (u8g.nextPage());

    // unsigned long currentMillis = millis();
    // if (currentMillis - previousMillis >= interval_limit) {
    //   Serial.println((String)millis() + ", " + position());
    //   previousMillis = currentMillis;
    // }
    // Serial.println( (String) "DATA,DATE,TIMER," + position() );
    // Serial.println(position());

    if (button2.isPressed()) {
      sensor_check_mode = false;
      while (button2.isPressed()) {
        delay(100);
      }
    }
    delay(10);
  }
}
void line_follow() {
  while (start_mode == true) {
    PID();
    if (menu_redraw_required != 0) {
      u8g.firstPage();
      do {
        u8g.setPrintPos(20, 8);
        u8g.print("Running....");
        u8g.setPrintPos(0, 24);
        u8g.print("Press ENTER(STOP)");
      } while (u8g.nextPage());
      menu_redraw_required = 0;
    }

    if (button2.isPressed()) {
      start_mode = false;
      while (button2.isPressed()) {
        delay(100);
      }
      menu_redraw_required = 1;
    }
    delay(10);  // Adjust delay as needed
  }
}
void line_follow2() {
  while (start_mode == true && !stopRobot) {
    PID();  // Perform PID control to follow the line

    // Check for cross section
    if (check_crossline()) {
      crossSectionCount++;  // Increment cross section counter
      if (crossSectionCount >= MAX_CROSS_SECTIONS) {
        stopRobot = true;        // Set flag to stop the robot
        motor_control(0, 0);     // Stop both motors
        display_stop_message();  // Optional: Display stop message on OLED
      } else {
        // Optional: Handle multiple cross sections if needed
      }
      delay(500);  // Debounce delay to prevent multiple detections
    }

    // Update OLED display
    if (menu_redraw_required != 0) {
      u8g.firstPage();
      do {
        u8g.setPrintPos(20, 8);
        u8g.print("Running....");
        u8g.setPrintPos(0, 24);
        u8g.print("Press ENTER(STOP)");
      } while (u8g.nextPage());
      menu_redraw_required = 0;
    }

    // Check if the stop button is pressed
    if (button2.isPressed()) {
      stopRobot = true;
      motor_control(0, 0);  // Stop both motors
      while (button2.isPressed()) {
        delay(100);  // Wait until button is released
      }
      menu_redraw_required = 1;
    }

    delay(10);  // Small delay for stability
  }

  // Reset flags after stopping
  if (stopRobot) {
    start_mode = false;
    stopRobot = false;
    crossSectionCount = 0;     // Reset counter if needed
    menu_redraw_required = 1;  // Redraw menu to reflect stopped state
  }
}
void line_follow3() {
  while (start_mode == true && !stopRobot) {
    PID();  // Perform PID control to follow the line

    // Check for cross section only if not currently processing one
    if (!intersectionDetected && check_crossline2()) {
      intersectionDetected = true;  // Set flag to indicate intersection is being processed
      crossSectionCount++;          // Increment cross section counter

      if (crossSectionCount >= MAX_CROSS_SECTIONS) {
        stopRobot = true;        // Set flag to stop the robot
        motor_control(0, 0);     // Stop both motors
        display_stop_message();  // Display stop message on OLED
      }

      lastIntersectionTime = millis();  // Record the time of detection
    }

    // Reset the intersectionDetected flag after cooldown period
    if (intersectionDetected && (millis() - lastIntersectionTime) > INTERSECTION_COOLDOWN) {
      intersectionDetected = false;
    }

    // Update OLED display
    if (menu_redraw_required != 0) {
      u8g.firstPage();
      do {
        u8g.setPrintPos(20, 8);
        u8g.print("Running....");
        u8g.setPrintPos(0, 24);
        u8g.print("Press ENTER(STOP)");
      } while (u8g.nextPage());
      menu_redraw_required = 0;
    }

    // Check if the stop button is pressed
    if (button2.isPressed()) {
      stopRobot = true;
      motor_control(0, 0);  // Stop both motors
      while (button2.isPressed()) {
        delay(100);  // Wait until button is released
      }
      menu_redraw_required = 1;
    }

    delay(10);  // Small delay for stability
  }

  // Reset flags after stopping
  if (stopRobot) {
    start_mode = false;
    stopRobot = false;
    crossSectionCount = 0;     // Reset counter if needed
    menu_redraw_required = 1;  // Redraw menu to reflect stopped state
  }
}
void line_follow4() {
  while (start_mode == true && !stopRobot) {
    PID();  // Perform PID control to follow the line

    unsigned long currentTime = millis();

    // Check for cross section only if not currently processing one
    if (!intersectionDetected && check_crossline()) {
      intersectionDetected = true;  // Set flag to indicate intersection is being processed
      crossSectionCount++;          // Increment cross section counter

      if (crossSectionCount >= MAX_CROSS_SECTIONS) {
        stopRobot = true;        // Set flag to stop the robot
        motor_control(0, 0);     // Stop both motors
        display_stop_message();  // Display stop message on OLED
      }

      lastIntersectionTime = currentTime;  // Record the time of detection
    }

    // Reset the intersectionDetected flag after cooldown period
    if (intersectionDetected && (currentTime - lastIntersectionTime) > INTERSECTION_COOLDOWN) {
      intersectionDetected = false;
    }

    // Update OLED display
    if (menu_redraw_required != 0) {
      u8g.firstPage();
      do {
        u8g.setPrintPos(20, 8);
        u8g.print("Running....");
        u8g.setPrintPos(0, 24);
        u8g.print("Press ENTER(STOP)");
      } while (u8g.nextPage());
      menu_redraw_required = 0;
    }

    // Check if the stop button is pressed
    if (button2.isPressed()) {
      stopRobot = true;
      motor_control(0, 0);  // Stop both motors
      while (button2.isPressed()) {
        delay(50);  // Wait until button is released
      }
      menu_redraw_required = 1;
    }

    // Small delay for stability (adjust as needed)
    delay(10);
  }

  // Reset flags after stopping
  if (stopRobot) {
    start_mode = false;
    stopRobot = false;
    crossSectionCount = 0;     // Reset counter if needed
    menu_redraw_required = 1;  // Redraw menu to reflect stopped state
  }
}


byte readSensor() {
  byte bitSensor = 0b00000000;  // Initialize byte to store sensor readings as bits

  for (int i = 0; i < 8; i++) {  // Loop through each sensor
    int sensorValue = analogRead(sensorPin[i]);

    // Compare sensor value with threshold and set/clear bit accordingly
    bitWrite(bitSensor, i, (sensorValue > ResultRef[i]) ? 1 : 0);
  }

  return bitSensor;  // Return the binary representation of sensor readings
}
int lastKnownPosition = 0;  // Variable to store the last known position

int position() {
  int count = 0;
  int m[8];  // Array size should match the number of sensors

  // Read sensors and populate m array
  for (int i = 0; i < 8; i++) {
    if (analogRead(sensorPin[i]) > ResultRef[i]) {
      m[i] = 1;
      count++;
    } else {
      m[i] = 0;
    }
  }

  // If all sensors detect white (no line)
  if (count == 0) {
    if (lastKnownPosition > 7500) {
      // If the last known position was positive (right turn), return a rightward value
      return 8000;
    } else if (lastKnownPosition < 1500) {
      // If the last known position was negative (left turn), return a leftward value
      return 1000;
    } else {
      // If the last known position was zero, return 0
      return 0;
    }
  }

  // If all sensors detect black (all on the line)
  if (count == 8) {
    // If all sensors detect the black line, return the middle position
    lastKnownPosition = 0;
    return 0;
  }

  // Calculate position based on sensor values
  int pos = 0;
  for (int i = 0; i < 8; i++) {
    pos += m[i] * (i + 1) * 1000;  // Adjust weight based on sensor position
  }
  pos = pos / count;  // Calculate average position

  // Update last known position and ensure it is within 0 to 8000 range
  lastKnownPosition = pos;

  return lastKnownPosition;  // Return the calculated position
}



int position2() {
  int count = 0;
  int m[8];  // Array size should match the number of sensors

  for (int i = 0; i < 8; i++) {  // Loop through all sensors
    if (analogRead(sensorPin[i]) > ResultRef[i]) {
      m[i] = 1;
      count++;
    } else {
      m[i] = 0;
    }
  }

  if (count == 0) {
    return 0;  // All sensor values are below threshold, return default value
  }

  // Calculate position based on sensor values
  int pos = 0;
  for (int i = 0; i < 8; i++) {
    pos += m[i] * (i + 1) * 1000;  // Adjust weight based on sensor position
  }
  pos = pos / count;  // Calculate average position

  return pos;
}
// Subroutines to get error values for several conditions
void check_Error() {
  byte bitSensor = readSensor();  //--> Calling the check_Sensor () subroutine
  int error = 0;
  //----------------------------------------Filling the error value
  if (bitSensor == 0b00000001) error = 13;
  if (bitSensor == 0b00000011) error = 11;
  if (bitSensor == 0b00000010) error = 9;
  if (bitSensor == 0b00000110) error = 7;
  if (bitSensor == 0b00000100) error = 5;
  if (bitSensor == 0b00001100) error = 3;
  if (bitSensor == 0b00001000) error = 1;
  if (bitSensor == 0b00011000) error = 0;
  if (bitSensor == 0b00010000) error = -1;
  if (bitSensor == 0b00110000) error = -3;
  if (bitSensor == 0b00100000) error = -5;
  if (bitSensor == 0b01100000) error = -7;
  if (bitSensor == 0b01000000) error = -9;
  if (bitSensor == 0b11000000) error = -11;
  if (bitSensor == 0b10000000) error = -13;


  //////////////////////////////////////////

  if (bitSensor == 0b11111111) error = 14;
  if (bitSensor == 0b00111111) error = 15;
  if (bitSensor == 0b00011111) error = 16;
  if (bitSensor == 0b00001111) error = 17;
  if (bitSensor == 0b11111110) error = 18;
  //----------------------------------------
}
void PID() {
  //int error = (analogRead(4) - analogRead(5));
  float error = position() - 4500;  //4000 best
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Kd * D) + (Ki * I);
  lsp = lfspeed + PIDvalue;
  rsp = lfspeed - PIDvalue;
  //PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);


  // lsp = lfspeed + PIDvalue;
  // rsp = lfspeed - PIDvalue;

  if (lsp > MAXSPEED) {
    lsp = MAXSPEED;
  }
  if (lsp < MINSPEED) {
    lsp = MINSPEED;
  }
  if (rsp > MAXSPEED) {
    rsp = MAXSPEED;
  }
  if (rsp < MINSPEED) {
    rsp = MINSPEED;
  }
  //int check = check_Error;
  if (position() == 0 || position() > 8000 || position() == -3692) {
    rsp = 0;
    lsp = 0;
  } else if (position() == 1000) {
    lsp = 0;
    rsp = MAXSPEED;

  } else if (position() == 8000) {
    lsp = MAXSPEED;
    rsp = 0;
  }

  // motor1.drive(lsp);
  //motor2.drive(rsp);
  motor_control(lsp, rsp);
  // if (check_crossline()) {
  //   while (check_crossline()) {
  //     line_follow();
  //   }
  //   dcount++;
  // }
  // if (dcount >= 2) {
  //   start_mode = false;
  // }
  // Serial.print(PIDvalue);
  // Serial.print("  ");
  // Serial.print(lsp);
  // Serial.print("  ");
  // Serial.print(rsp);
  // Serial.print("  ");
  // Serial.print(position());
  // delay(250);
  // Serial.println();
  previousError = error;
}
void PID2() {
  //int error = (analogRead(4) - analogRead(5));
  float error = position() - 4500;  //4000 best
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Kd * D) + (Ki * I);
  lsp = lfspeed + PIDvalue;
  rsp = lfspeed - PIDvalue;
  //PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);


  // lsp = lfspeed + PIDvalue;
  // rsp = lfspeed - PIDvalue;

  if (lsp > MAXSPEED) {
    lsp = MAXSPEED;
  }
  if (lsp < MINSPEED) {
    lsp = MINSPEED;
  }
  if (rsp > MAXSPEED) {
    rsp = MAXSPEED;
  }
  if (rsp < MINSPEED) {
    rsp = MINSPEED;
  }
  //int check = check_Error;
  if (position() == 0 || position() > 8000 || position() == -3692) {
    rsp = 0;
    lsp = 0;
  } else if (position() == 1000) {
    lsp = 0;
    rsp = MAXSPEED;

  } else if (position() == 8000) {
    lsp = MAXSPEED;
    rsp = 0;
  }
  // motor1.drive(lsp);
  //motor2.drive(rsp);
  // motor_control(lsp, rsp);
  // Serial.print(PIDvalue);
  // Serial.print("  ");
  // Serial.print(lsp);
  // Serial.print("  ");
  // Serial.print(rsp);
  // Serial.print("  ");
  // Serial.print(position());
  // delay(250);
  // Serial.println();
  previousError = error;
}
byte sensor_inp(byte mask) {
  // Read the sensor values as a binary byte
  byte sensor = readSensor();

  // Apply the mask to filter the relevant bits
  sensor &= mask;

  // Return the masked sensor value
  return sensor;
}
// Subroutine for displaying sensors in binary form on an OLED Display
void printBinaryByte(byte value) {
  for (byte mask = 0x01; mask; mask <<= 1) {
    u8g.print((mask & value) ? '1' : '0');
  }
}


/***********************************************************************/
/* Cross line detection processing */
/* Return values: 0: no cross line, 1: cross line */
/***********************************************************************/
int check_crossline(void) {
  byte b;
  int ret;

  ret = 0;
  b = sensor_inp(MASK2_2);

  // Detect cross line based on sensor pattern
  // Adjust the mask and condition as per your sensor arrangement
  if (b == MASK2_2) {
    ret = 1;
  }

  return ret;
}
/***********************************************************************
  /* Cross line detection processing
  /* Return values: 0: no cross line, 1: cross line
  /***********************************************************************/
int check_crossline2(void) {
  byte b = readSensor();  // Read all sensor inputs

  // Example condition: if the central four sensors detect the line
  if ((b & 0b00111100) == 0b01111110) {  // Adjust mask as per your sensor layout
    return 1;                            // Cross line detected
  }
  return 0;
}

/***********************************************************************/
/* Right half line detection processing */
/* Return values: 0: not detected, 1: detected */
/***********************************************************************/
int check_rightline(void) {
  byte b;
  int ret;

  ret = 0;
  b = sensor_inp(MASK4_4);
  if (b == B00000111 || b == B00001111 || b == B00011111) {
    ret = 1;
  }
  return ret;
}

/***********************************************************************/
/* Left half line detection processing */
/* Return values: 0: not detected, 1: detected */
/***********************************************************************/
int check_leftline(void) {
  byte b;
  int ret;

  ret = 0;
  b = sensor_inp(MASK4_4);
  if (b == B11100000 || b == B11110000 || b == B11111000) {
    ret = 1;
  }
  return ret;
}
void left() {
  // motor_control(-lfspeed, lfspeed);  // Start turning left
  delay(200);  // Initial delay for consistent turn

  while (position() != 4500) {  // Continue turning left until the sensor detects the line
    //motor_control(-lfspeed, lfspeed);
    motor_control(0, lfspeed);
  }

  motor_control(0, 0);  // Stop motors
  delay(50);            // Small delay to stabilize
}

void right() {
  //  motor_control(lfspeed, -lfspeed);  // Start turning right
  delay(200);  // Initial delay for consistent turn

  while (position() != 4500) {  // Continue turning right until the sensor detects the line
    //motor_control(lfspeed, -lfspeed);
    motor_control(lfspeed, 0);
  }

  motor_control(0, 0);  // Stop motors
  delay(50);            // Small delay to stabilize
}

void uturn() {
  motor_control(-lfspeed * 0.8, lfspeed);  // Start U-turn (left turn with different speeds)
  delay(400);                              // Initial delay for consistent U-turn

  while (position() != 4500) {  // Continue U-turn until the sensor detects the line
    motor_control(-lfspeed * 0.8, lfspeed);
  }

  motor_control(0, 0);  // Stop motors
  delay(50);            // Small delay to stabilize
}

// Modified detect_junction function
JunctionType detect_junction() {
  byte sensor = readSensor();
  static byte prevSensor = 0;
  static unsigned long lastDetection = 0;

  // Debounce detection
  if (millis() - lastDetection < 300) return NONE;

  // Sharp turn detection (L-shape pattern)
  if ((sensor & 0b11110000) == 0b11110000 ||  // Right sharp turn
      (sensor & 0b00001111) == 0b00001111) {  // Left sharp turn
    lastDetection = millis();
    return SHARP_TURN;
  }

  // T-Junction (all center sensors)
  if ((sensor & 0b00111100) == 0b00111100) {
    lastDetection = millis();
    return T_JUNCTION;
  }

  // Y-Junction (split detection)
  if ((sensor & 0b11000011) == 0b11000011) {
    lastDetection = millis();
    return Y_JUNCTION;
  }

  return NONE;
}



// New state machine implementation
void line_follow5() {
  while (start_mode && !stopRobot) {
    JunctionType detected = detect_junction();

    switch (navState) {
      case FOLLOWING_LINE:
        PID();
        if (detected != NONE) handle_junction_detection(detected);
        break;

      case PRE_TURN:
        execute_pre_turn();
        break;

      case TURNING:
        execute_turn();
        break;

      case LINE_SEARCH:
        search_for_line();
        break;
    }

    update_display();
    check_stop_button();
  }
}

void handle_junction_detection(JunctionType detected) {
  // Always prioritize left turns for T/Y junctions
  if (detected == T_JUNCTION || detected == Y_JUNCTION) {
    turnDirection = 1;  // Left turn
    navState = PRE_TURN;
    stateStartTime = millis();
  } else if (detected == SHARP_TURN) {
    // Determine turn direction from sensor pattern
    byte sensor = readSensor();
    turnDirection = (sensor & 0b11110000) ? -1 : 1;  // Right:1, Left:-1
    navState = PRE_TURN;
    stateStartTime = millis();
  }
}

void execute_pre_turn() {
  // Move straight for a short distance
  motor_control(lfspeed, lfspeed);

  if (millis() - stateStartTime > PRE_TURN_DISTANCE) {
    // Stop briefly before turning
    motor_control(0, 0);
    delay(200);

    // Start turning
    navState = TURNING;
    stateStartTime = millis();
  }
}

void execute_turn() {
  // Turn in place
  motor_control(turnDirection * TURN_SPEED, -turnDirection * TURN_SPEED);

  // Check for line reacquisition
  if (position() != 0) {
    navState = LINE_SEARCH;
    stateStartTime = millis();
  }
}

void search_for_line() {
  // Slow down while searching
  motor_control(turnDirection * (TURN_SPEED), -turnDirection * (TURN_SPEED));

  // Check if line is properly aligned
  if (abs(position() - 4500) < 500) {  // Within ±500 from center
    navState = FOLLOWING_LINE;
    motor_control(lfspeed, lfspeed);
  }

  // Timeout safety
  if (millis() - stateStartTime > LINE_REACQUIRE_TIMEOUT) {
    navState = FOLLOWING_LINE;
    motor_control(0, 0);  // Full stop on timeout
  }
}
void check_stop_button() {
  // Check if the stop button is pressed
  if (button2.isPressed()) {
    stopRobot = true;
    motor_control(0, 0);  // Stop both motors
    while (button2.isPressed()) {
      delay(50);  // Wait until button is released
    }
    menu_redraw_required = 1;
  }
}

// Add this helper function
void update_display() {
  if (menu_redraw_required) {
    u8g.firstPage();
    do {
      u8g.setPrintPos(20, 8);
      u8g.print(navState == FOLLOWING_LINE ? "Following" : "Turning");
      u8g.setPrintPos(0, 24);
      u8g.print("Press ENTER(STOP)");
    } while (u8g.nextPage());
    menu_redraw_required = 0;
  }
}