
#include <stdio.h>
#include <driverlib.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <stdlib.h>
#include<HX711.h>

/* LCD Screen libraries*/
#include <LCD_screen.h>
#include <LCD_screen_font.h>
#include <LCD_utilities.h>
#include <Screen_HX8353E.h>
#include <Terminal12e.h>
#include <Terminal6e.h>
#include <Terminal8e.h>
Screen_HX8353E myScreen;
volatile uint16_t airTime = 0;
volatile uint16_t counter = 0;
int landed = 1;
int prevDist = 0;
int prevDisY;
volatile int angleCount;
int angle;
int goodStride;
char buf[3];
int IO_button = 0;
int buffer = 0;
int state = 0;
int count = 0;
int num = 0;
int gesture = 0;
int flag = 0;
int16_t accelX[2] = {0, 0};
int16_t accelY[2] = {0, 0};
int16_t veloX[2] = {0, 0};
int16_t veloY[2] = {0, 0};
int16_t disX[2] = {0, 0};
int16_t disY[2] = {0, 0};
int16_t buffX[10];
int16_t buffY[10];
int16_t resultsBuffer[2];
int t1;
int t2;

const int LOADCELL_DOUT_PIN = 23;
const int LOADCELL_SCK_PIN = 24;
HX711 scale;

void drawAccelData(void);


// your network name also called SSID
char ssid[] = "";
// your network password
char password[] = "";
// MQTTServer to use
char server[] = "io.adafruit.com";


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message for topic ");
  Serial.print(topic);
  Serial.print("with length ");
  Serial.println(length);
  Serial.println("Message:");
  Serial.write(payload, length);
  Serial.println();
}

WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);

void LCD_init(){
  
  myScreen.begin();

  /* Let's make a title*/
  myScreen.gText(8, 5, "Acceleration Data");
  myScreen.gText(8, 35, "Velocity Data");
  myScreen.gText(8, 65, "Displacement Data");
  myScreen.gText(8, 95, "Gesture Detected");
                  
}

void ADC_init(){
  /* Configures Pin 4.0, 4.2, and 6.1 ad ADC inputs */
  // ACC Z = P4.2
  // ACC Y = P4.0
  // ACC X = P6.1
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

  ADC14_registerInterrupt(ADC14_IRQHandler);

  /* Initializing ADC (ADCOSC/64/8) */
  ADC14_enableModule();
  ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

  /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A11, A13, A14)  with no repeat)
   * with internal 2.5v reference */
  ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
  ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

  ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

  ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);


  ADC14_setResolution(ADC_10BIT); //IMPORTANT -> This seemed to give me the least noisey values (8 bit res was too small though)

  /* Enabling the interrupt when a conversion on channel 2 (end of sequence)
   *  is complete and enabling conversions */
   ADC14_enableInterrupt(ADC_INT2);

  /* Enabling Interrupts */
  Interrupt_enableInterrupt(INT_ADC14);
  Interrupt_enableMaster();

  /* Setting up the sample timer to automatically step through the sequence
   * convert.*/
  ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

  /* Triggering the start of the sample */
  ADC14_enableConversion();
  ADC14_toggleConversionTrigger();
}

void drawAccelData(void){
  myScreen.gText(40, 15,  "X: " + String(accelX[1]));
  myScreen.gText(40, 25,  "Y: " + String(accelY[1]));
  myScreen.gText(40, 45,  "X: " + String(veloX[1]));
  myScreen.gText(40, 55,  "Y: " + String(veloY[1]));
  myScreen.gText(40, 75,  "X: " + String(disX[1]));
  myScreen.gText(40, 85,  "Y: " + String(disY[1]));
  myScreen.gText(40, 105,  "L: " + String(landed));
  myScreen.gText(40, 115,  "A: " + String(angle));
  myScreen.gText(70, 115,  "C: " + String(goodStride));
  
}

void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);
    num++;
    /* ADC_MEM2 conversion completed */
    if(status & ADC_INT2 && flag == 0)
    {

        // if ([DATA IS NOT BEING PUBLISHED TO SERVER])
        
        prevDisY = disY[1];
        /* Store ADC14 conversion results */
        resultsBuffer[0] = ADC14_getResult(ADC_MEM0); // X Accelerometer
        resultsBuffer[1] = ADC14_getResult(ADC_MEM1); // Y Accelerometer
       
    
        /* Below implement logic for a smoothing filter for Acc readings, numReadings = 10 */
        buffX[buffer] = resultsBuffer[0];
        buffY[buffer] = resultsBuffer[1];
        buffer++;
        if(buffer>=10){
          buffer=0;
        }
        for(int i=0; i<10;i++){
          accelX[1] += buffX[i];
          accelY[1] += buffY[i];
        }
        accelX[1] /= 10;
        accelY[1] /= 10;
       
       /************************************************************************************/

        // Calculate Velocity Readings 
        veloX[1] = (10*(accelX[1]+accelX[0]))/(2*8.5);
        veloY[1] = (10*(accelY[1]+accelY[0]))/(2*8.5);    

        // Calculate Displacement Readings
        disX[1] = (10*(veloX[1]+veloX[0]))/(2*8.5);
        disY[1] = (10*(veloY[1]+veloY[0]))/(2*8.5);

       counter++;
       if(disY[1] < 235){
        if(landed == 0){
          landed = 1;
          prevDist = disY[1];
        }else if(counter > 10){
          landed = 0;
          counter = 0;
          if(prevDisY < 220){
            goodStride++;
          }
        }
          
       }
       if(prevDist != 0){
          angleCount++;
       }
       if(angleCount > 10){
          angle = prevDist - disY[1];
       }
       
      
       
        // Draw accelerometer data on display 
        drawAccelData();
      
        accelX[1] = 0;
        accelY[1] = 0;
        flag = 1;
       
    }
}

void setup() {

  WDT_A_hold(WDT_A_BASE);

  Serial.begin(115200);
  flag = 0;
  
  // Start Ethernet with the build in MAC Address
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to Network named: ");
  // print the network name (SSID);
  Serial.println(ssid); 
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED) {
    // print dots while we wait to connect
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nYou're connected to the network");
  /* Initialize all variables to 0 here  */


  /***************************************/

  
  /* Initialize LCD and make some titles */
  LCD_init();
  /* Initialize ADC *SEE THIS FUNCTION IMPORTANT LINES ARE HIGHLIGHTED* */
  ADC_init();
}


void loop() {

 if(num >=200){     
  flag = 1;
  if (!client.connected()) {
    Serial.println("Disconnected. Reconnecting....");

    if(!client.connect("energiaClient", "Dvlta", "key")) {
      Serial.println("Connection failed");
    } else {
      Serial.println("Connection success");
      if(client.subscribe("Dvlta/feeds/final.angle")) {
            Serial.println("Subscription successfull");
          }
    }
   } 
  char cstr[4];
  itoa(goodStride, cstr, 10);
  char cstr2[4];
  itoa(angle, cstr2, 10);
//Serial.println(String(cstr)+"A");


    if(client.publish("Dvlta/feeds/final.strides", cstr) && client.publish("Dvlta/feeds/final.angle", cstr2)) {//cstr

      Serial.println("Publish success");
    } else {
      Serial.println("Publish failed");
    }
  num = 0;
  flag = 0;
 }else{
  flag = 0;
 }
}
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
  
  
