
#include <bluefruit.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <SPI.h>
#include <Wire.h>

#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);

typedef struct
{
  char name[16+1];

  uint16_t conn_handle;

  // Each prph need its own bleuart client service
  BLEClientUart bleuart;

  float currentTemp = 0;
  float maxTemp = 1;
  float minTemp = 1000;
  int measurements = 0;
  float sum = 0; 
  float avgTemp;
  int threshold = 45;
  bool thresholdBreached;
} prph_info_t;

prph_info_t prphs[BLE_MAX_CONNECTION];
//prph_info_t prphs[2];

// Software Timer for blinking the RED LED
SoftwareTimer blinkTimer;
uint8_t connection_num = 0; // for blink pattern

void setup() 
{
  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // for nrf52840 with native usb

  // Initialize blinkTimer for 100 ms and start it
  blinkTimer.begin(100, blink_timer_callback);
  blinkTimer.start();

  Serial.println("Bluefruit52 Central Multi BLEUART Example");
  Serial.println("-----------------------------------------\n");
  
  // Initialize Bluefruit with max concurrent connections as Peripheral = 0, Central = 4
  // SRAM usage required by SoftDevice will increase with number of connections
  Bluefruit.begin(0, 3);

  // Set Name
  Bluefruit.setName("RV Fridge Sensor Central");
  
  // Init peripheral pool
  for (uint8_t idx=0; idx<BLE_MAX_CONNECTION; idx++)
  {
    // Invalid all connection handle
    prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;
    
    // All of BLE Central Uart Serivce
    prphs[idx].bleuart.begin();
    prphs[idx].bleuart.setRxCallback(bleuart_rx_callback);
  }

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service in advertising
   * - Don't use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);       // Don't request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds

  display.begin(0x3C, true);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();
  display.setRotation(1);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP); 
}

/**
 * Callback invoked when scanner picks up an advertising packet
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with bleuart service advertised  
  // Connect to the device with bleuart service in advertising packet
  Bluefruit.Central.connect(report);
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  // Find an available ID to use
  int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);

  // Eeek: Exceeded the number of connections !!!
  if ( id < 0 ) return;
  
  prph_info_t* peer = &prphs[id];
  peer->conn_handle = conn_handle;
  
  Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name)-1);

  Serial.print("Connected to ");
  Serial.println(peer->name);

  Serial.print("Discovering BLE UART service ... ");

  if ( peer->bleuart.discover(conn_handle) )
  {
    Serial.println("Found it");
    Serial.println("Enabling TXD characteristic's CCCD notify bit");
    peer->bleuart.enableTXD();

    Serial.println("Continue scanning for more peripherals");
    Bluefruit.Scanner.start(0);

    Serial.println("Enter some text in the Serial Monitor to send it to all connected peripherals:");
  } else
  {
    Serial.println("Found ... NOTHING!");

    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }  

  connection_num++;
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  connection_num--;

  // Mark the ID as invalid
  int id  = findConnHandle(conn_handle);

  // Non-existant connection, something went wrong, DBG !!!
  if ( id < 0 ) return;

  // Mark conn handle as invalid
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;

  Serial.print(prphs[id].name);
  Serial.println(" disconnected!");
}

/**
 * Callback invoked when BLE UART data is received
 * @param uart_svc Reference object to the service where the data 
 * arrived.
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  float reading;
  // uart_svc is prphs[conn_handle].bleuart
  uint16_t conn_handle = uart_svc.connHandle();

  int id = findConnHandle(conn_handle);
  prph_info_t* peer = &prphs[id];
  
  // Print sender's name
  Serial.printf("[From %s]: ", peer->name);

  // Read
  while ( uart_svc.available() )
  {
    // default MTU with an extra byte for string terminator
    char buf[20+1] = { 0 };
    
    if ( uart_svc.read(buf,sizeof(buf)-1) )
    {
      Serial.println(buf);
      reading = atof(buf);
    }
  }
  //Serial.print(" Last Reading: ");
  //Serial.println(reading);

  prphs[conn_handle].currentTemp = reading;

  if(reading < prphs[conn_handle].minTemp){
    prphs[conn_handle].minTemp = reading;
    //Serial.println("New minTemp found!");
  }  
  if(reading > prphs[conn_handle].maxTemp){
    prphs[conn_handle].maxTemp = reading;
    //Serial.println("New maxTemp found!");
  }
  if(reading > prphs[conn_handle].threshold){
    prphs[conn_handle].thresholdBreached = true;
    //Serial.println("Warning: Threshold Breached!");
  }

  prphs[conn_handle].measurements++;
  prphs[conn_handle].sum += reading;
  prphs[conn_handle].avgTemp = prphs[conn_handle].sum / prphs[conn_handle].measurements;
}

void loop()
{
  /* First check if we are connected to any peripherals
  if ( Bluefruit.Central.connected() )
  {
    // default MTU with an extra byte for string terminator
    char buf[20+1] = { 0 };
    
    // Read from HW Serial (normally USB Serial) and send to all peripherals
    if ( Serial.readBytes(buf, sizeof(buf)-1) )
    {
      sendAll(buf);
    }
  }*/

  /* PUSHING BUTTON A RESETS AVERAGE
  if(!digitalRead(BUTTON_A)) sum = measurements = 0;
  // PUSHING BUTTON B RESETS MIN/MAX
  if(!digitalRead(BUTTON_B)) {
    minTemp = 1000;
    maxTemp = 1;
  }  
  // PUSHING BUTTON C RESETS HIGH TEMP WARNING
  if(!digitalRead(BUTTON_C)) thresholdBreached = false;
  */

  // TESTING DISPLAY
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(0,0);  
    
  // Cycle through display of each sensors' readings
  // Print out to display
  for(prph_info_t sensor: prphs) {
    if(sensor.currentTemp != 0) {
      display.clearDisplay();
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
      display.setCursor(0,0);
      display.print("Current Temp: "); display.print(sensor.currentTemp); display.println(" F");
      Serial.println();
      Serial.print("Current Temp: "); Serial.print(sensor.currentTemp); Serial.println(" F");
      display.println("");
  
      // Print average, minimum & maximum temperatures
      display.print("Average Temp: "); display.print(sensor.avgTemp); display.println(" F");
      Serial.print("Average Temp: "); Serial.print(sensor.avgTemp); Serial.println(" F");
      display.println("");
      display.print("Minimum Temp: "); display.print(sensor.minTemp); display.println(" F");
      Serial.print("Minimum Temp: "); Serial.print(sensor.minTemp); Serial.println(" F");
      display.print("Maximum Temp: "); display.print(sensor.maxTemp); display.println(" F");
      Serial.print("Maximum Temp: "); Serial.print(sensor.maxTemp); Serial.println(" F");

      Serial.print("Measurements: "); Serial.println(sensor.measurements);

      // Print threshold breached warning if true
      if(sensor.thresholdBreached) {
        display.println("");
        Serial.println("");
        display.setTextColor(SH110X_BLACK, SH110X_WHITE);
        display.print("! HIGH TEMP WARNING !");
        Serial.println("! HIGH TEMP WARNING !");
      }
      else {
        display.println("");
        Serial.println("");
        display.print("No breaches detected.");
        Serial.println("No breaches detected.");
      }
      display.display();
      delay(5000);
    }
    else {
      // No readings, skip
    }
  }
}

/**
 * Find the connection handle in the peripheral array
 * @param conn_handle Connection handle
 * @return array index if found, otherwise -1
 */
int findConnHandle(uint16_t conn_handle)
{
  for(int id=0; id<BLE_MAX_CONNECTION; id++)
  {
    if (conn_handle == prphs[id].conn_handle)
    {
      return id;
    }
  }

  return -1;  
}

/**
 * Software Timer callback is invoked via a built-in FreeRTOS thread with
 * minimal stack size. Therefore it should be as simple as possible. If
 * a periodically heavy task is needed, please use Scheduler.startLoop() to
 * create a dedicated task for it.
 * 
 * More information http://www.freertos.org/RTOS-software-timer.html
 */
void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;

  // Period of sequence is 10 times (1 second). 
  // RED LED will toggle first 2*n times (on/off) and remain off for the rest of period
  // Where n = number of connection
  static uint8_t count = 0;

  if ( count < 2*connection_num ) digitalToggle(LED_RED);
  if ( count % 2 && digitalRead(LED_RED)) digitalWrite(LED_RED, LOW); // issue #98

  count++;
  if (count >= 10) count = 0;
}
