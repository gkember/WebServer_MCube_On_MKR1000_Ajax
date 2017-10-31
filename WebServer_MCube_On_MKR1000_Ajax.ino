/*
  An Arduino code to provide a generic basic web server that produces a 
  webpage that displays values of some 3 exemplary variables called X,Y,Z. 
  It allows AJAX CORS. To use it:
  1- Enter the SSID and PWD below for your network.
  2- Upload to the MKR1000.
  3- Find the IP of this MKR1000 via the Serial Monitor. If the
     WiFi connection is successful the IP should be printed in 
     the Serial Monitor in few seconds.
  4- Go to a browser on a computer on the same network, and enter any of these
     URLs (REST style):
     http://MKR1000IPAddress/XYZ 
     http://MKR1000IPAddress/X
     http://MKR1000IPAddress/Y
     http://MKR1000IPAddress/Z    
  One can change the server to accept different URLs and serve other variables
  and different web pages. The variables would be measurements from the Arduino
  sensors.
  S.M.
 */
#include <SPI.h>
#include <WiFi101.h>
#include <MC3672.h> // mc3672 library included here for simplicity

//Mcube stuff
MC3672 MC3672_acc = MC3672();
int Ts            = 1000;//sampling period

char ssid[] = "";//  your network SSID (name)
char pass[] = "";//               // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
             
int status = WL_IDLE_STATUS;
WiFiServer server(80);          
String HTTPResponseHeaders = "";//Assuming all responses will use the same headers 

void setup() {
  Serial.begin(9600);      // initialize serial communication
   MC3672_acc.start();
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);       // don't continue
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();                           // start the web server on port 80
  printWiFiStatus();                        // you're connected now, so print out the 
  
  /* HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
     and a content-type so the client knows what's coming, then a blank line: */
  HTTPResponseHeaders += "HTTP/1.1 200 OK\n";
  HTTPResponseHeaders += "Access-Control-Allow-Origin: *\n";    //This is needed for AJAX CORS
  HTTPResponseHeaders += "Access-Control-Allow-Methods: GET\n"; //This is needed for AJAX CORS
  HTTPResponseHeaders += "Content-type:text/html\n";
  HTTPResponseHeaders += "\n";
}

void loop() {
  WiFiClient client = server.available();   // listen for incoming clients
  
  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine  = "";                // make a String to hold incoming data from the client
    String HTTPRequest  = "";
    String HTTPResponse = HTTPResponseHeaders + "";
    
    boolean XYZRequested = false;
    
    /*This loop is meant to read the full HTTP request. Note the end of a request is marked by two \n */
    /* client.connected() is true as long as the client is connected OR disconnected but still 
       some data is in the stream WiFiClient inherits from Stream */
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        HTTPRequest += c;
        //Serial.write(c);                    // print it out the serial monitor
        /* Check if we hit the end of the request */
        if(HTTPRequest.endsWith("\r\n\r\n")) {//This is the end of HTTPRequests
          break; //We have received the whole request
        }      
      }      
    }

    /* Now process the request and send the corresponding response
       Structure of the request  GET /XYZ HTTP/1.1
                                 Host: 172.20.10.11 ...
    */
    int slashidx   = HTTPRequest.indexOf("/"); //Locate the /... 
    int HTTPidx    = HTTPRequest.indexOf("HTTP/"); //Locate HTTP
    String command = HTTPRequest.substring(slashidx,HTTPidx); //GET
    command.trim();//Remove extra spaces
    
    Serial.println("**************** Recvd HTTP Request ********************");
    Serial.println(HTTPRequest);
    Serial.println("**************** Resource requested ********************");
    Serial.println(command);

    HTTPResponse += ("<h2 style=\"color:green\">" + String("Command:") + command + "</h2> </br>\n");
    
    MC3672_acc_t rawAccel = MC3672_acc.readRawAccel();
    
    if(command == "/XYZ") {
       HTTPResponse += "X=" + String(rawAccel.XAxis_g) + " ";
       HTTPResponse += "Y=" + String(rawAccel.YAxis_g) + " ";
       HTTPResponse += "Z=" + String(rawAccel.ZAxis_g) + " "; 
    } else if (command == "/X"){ 
       HTTPResponse += "X=" + String(rawAccel.XAxis_g) + " ";
    } else if (command == "/Y") {
       HTTPResponse += "Y=" + String(rawAccel.YAxis_g) + " ";
    } else if (command == "/Z") {
       HTTPResponse += "Z=" + String(rawAccel.ZAxis_g) + " ";
    } else {
       HTTPResponse +=   "<h1 style=\"color:red\">" + String("Wrong request. Please use </br>" ) + 
                          "http://MKR1000IPAddress/XYZ </br> or </br>" + 
                          String("http://MKR1000IPAddress/X </br> or </br>")  + " ... </h1>";
    }
    //Send the HTTPResponse to the client    
    client.println(HTTPResponse);
    //Request-Response transaction done, close the connection:
    client.stop();
    Serial.println("client disonnected");
    Serial.println("*********************************************************");
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

  
#ifdef MC3672_CFG_BUS_I2C 
  #define MC3672_CFG_I2C_ADDR   (0x4C)
#endif
#define MC3672_CFG_MODE_DEFAULT     MC3672_MODE_STANDBY
#define MC3672_CFG_SAMPLE_RATE_CWAKE_DEFAULT    MC3672_CWAKE_SR_DEFAULT_54Hz
#define MC3672_CFG_SAMPLE_RATE_SNIFF_DEFAULT    MC3672_SNIFF_SR_7Hz
#define MC3672_CFG_RANGE_DEFAULT    MC3672_RANGE_8G
#define MC3672_CFG_RESOLUTION_DEFAULT   MC3672_RESOLUTION_14BIT
#define MC3672_CFG_ORIENTATION_MAP_DEFAULT      ORIENTATION_TOP_RIGHT_UP

uint8_t CfgRange, CfgResolution;  

// Read register bit
bool MC3672::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
}

// Write register bit
void MC3672::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if (state)
        value |= (1 << pos);
    else
        value &= ~(1 << pos);

    writeRegister8(reg, value);
}

// Read 8-bit from register
uint8_t MC3672::readRegister8(uint8_t reg)
{
    uint8_t value;
  
#ifdef MC3672_CFG_BUS_I2C   
    Wire.beginTransmission(MC3672_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);    //endTransmission but keep the connection active
    Wire.requestFrom(MC3672_CFG_I2C_ADDR, 1);   //Once done, bus is released by default
    value = Wire.read();
#else  //Reads an 8-bit register with the SPI port.
  digitalWrite(chipSelectPin, LOW);   //Set active-low CS low to start the SPI cycle
  SPI.transfer(reg | 0x80 | 0x40);  //Send the register address
  value = SPI.transfer(0x00);     //Read the value from the register
  digitalWrite(chipSelectPin, HIGH);  //Raise CS
#endif  

    return value;
}          

// Write 8-bit to register
void MC3672::writeRegister8(uint8_t reg, uint8_t value)
{
#ifdef MC3672_CFG_BUS_I2C 
    Wire.beginTransmission(MC3672_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
#else
  digitalWrite(chipSelectPin, LOW);   //Set active-low CS low to start the SPI cycle
  SPI.transfer(reg | 0x40);     //Send the register address
  SPI.transfer(value);        //Send value to write into register
  digitalWrite(chipSelectPin, HIGH);  //Raise CS
#endif  
}

// Read 16-bit from register
int16_t MC3672::readRegister16(uint8_t reg)
{
    int16_t value;

#ifdef MC3672_CFG_BUS_I2C     
    Wire.beginTransmission(MC3672_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(MC3672_CFG_I2C_ADDR, 2);
    while(!Wire.available()) {};
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
#else     
  digitalWrite(chipSelectPin, LOW);   //Set active-low CS low to start the SPI cycle
  SPI.transfer(reg | 0x80 | 0x40);  //Send the register address
    uint8_t vha = SPI.transfer(0x00);     //Send a value of 0 to read the first byte returned
    uint8_t vla = SPI.transfer(0x00);     //Send a value of 0 to read the second byte returned
  digitalWrite(chipSelectPin, HIGH);  //Raise CS
#endif    
  
  value = vha << 8 | vla;
  
    return value;
}

// Write 16-bit to register
void MC3672::writeRegister16(uint8_t reg, int16_t value)
{
#ifdef MC3672_CFG_BUS_I2C   
    Wire.beginTransmission(MC3672_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)value);
    Wire.endTransmission();
#else 
  digitalWrite(chipSelectPin, LOW);     //Set active-low CS low to start the SPI cycle
  SPI.transfer(reg | 0x40);       //Send the register address 
  SPI.transfer((uint8_t)(value >> 8));      //Send value to write into register
  SPI.transfer((uint8_t)value);       //Send value to write into register
  digitalWrite(chipSelectPin, HIGH);    //Raise CS
#endif    
}

// Repeated Read Byte(s) from register
void MC3672::readRegisters(uint8_t reg, byte *buffer, uint8_t len)
{
#ifdef MC3672_CFG_BUS_I2C   
    Wire.beginTransmission(MC3672_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);      //endTransmission but keep the connection active
    Wire.requestFrom(MC3672_CFG_I2C_ADDR, len);   //Ask for bytes, once done, bus is released by default

    while(Wire.available() < len);      //Hang out until we get the # of bytes we expect
    for(int x = 0 ; x < len ; x++)
    buffer[x] = Wire.read();
#else
  digitalWrite(chipSelectPin, LOW);     //Set active-low CS low to start the SPI cycle
  SPI.transfer(reg | 0x80 | 0x40);    //send the device the register you want to read
  
  for(int x = 0 ; x < len ; x++)                //Prepare to clock in the data to be read
    buffer[x] = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, HIGH);    //Raise CS
#endif  

}
  
//Set the operation mode  
void MC3672::SetMode(MC3672_mode_t mode)
{
    uint8_t value;
    value = readRegister8(MC3672_REG_MODE_C);
    value &= 0b11110000;
    value |= mode;
    writeRegister8(MC3672_REG_MODE_C, value);
}

//Set the range control
void MC3672::SetRangeCtrl(MC3672_range_t range)
{
    uint8_t value;    
    CfgRange = range;
    SetMode(MC3672_MODE_STANDBY);
    value = readRegister8(MC3672_REG_RANGE_C);
    value &= 0b00000111;
    value |= (range << 4)&0x70 ;
    writeRegister8(MC3672_REG_RANGE_C, value);
}

//Initial reset
void MC3672::reset()
{
    writeRegister8(0x10, 0x01);
  
  delay(10);
  
  writeRegister8(0x24, 0x40);

  delay(50);  
  
  writeRegister8(0x09, 0x00); 
  delay(10);
  writeRegister8(0x0F, 0x42);
  delay(10);
  writeRegister8(0x20, 0x01);
  delay(10);
  writeRegister8(0x21, 0x80);
  delay(10);
  writeRegister8(0x28, 0x00);
  delay(10);
  writeRegister8(0x1a, 0x00); 
  
  delay(50);  
  
  uint8_t _bRegIO_C = 0;

  _bRegIO_C = readRegister8(0x0D);

  #ifdef MC3672_CFG_BUS_I2C
    _bRegIO_C &= 0x3F;
    _bRegIO_C |= 0x40;
  #else   
    _bRegIO_C &= 0x3F;
    _bRegIO_C |= 0x80;
  #endif

  writeRegister8(0x0D, _bRegIO_C);
  
  delay(50);

  writeRegister8(0x10, 0x01);
  
  delay(10);
}

//Set Sniff Analog Gain
void MC3672::SetSniffAGAIN(MC3672_gain_t gain)
{
    writeRegister8(0x20, 0x00);
    uint8_t value;
    value = readRegister8(MC3672_REG_GAIN);
    value &= 0b00111111;
    value |= (gain << 6);
    writeRegister8(MC3672_REG_GAIN, value);
}

//Set CWake Analog Gain
void MC3672::SetWakeAGAIN(MC3672_gain_t gain)
{
    writeRegister8(0x20, 0x01);
    uint8_t value;
    value = readRegister8(MC3672_REG_GAIN);
    value &= 0b00111111;
    value |= (gain << 6);
    writeRegister8(MC3672_REG_GAIN, value);
}

//Set the resolution control
void MC3672::SetResolutionCtrl(MC3672_resolution_t resolution)
{
  uint8_t value;
  CfgResolution = resolution;
  SetMode(MC3672_MODE_STANDBY);
  value = readRegister8(MC3672_REG_RANGE_C);
  value &= 0b01110000;
  value |= resolution;
  writeRegister8(MC3672_REG_RANGE_C, value);
}

//Set the sampling rate
void MC3672::SetCWakeSampleRate(MC3672_cwake_sr_t sample_rate)
{
  uint8_t value;
  SetMode(MC3672_MODE_STANDBY);
  value = readRegister8(MC3672_REG_WAKE_C); 
  value &= 0b00000000;
  value |= sample_rate;
  writeRegister8(MC3672_REG_WAKE_C, value);
}

//Get the output sampling rate
MC3672_cwake_sr_t MC3672::GetCWakeSampleRate(void)
{
  /* Read the data format register to preserve bits */
  uint8_t value;
  value = readRegister8(MC3672_REG_WAKE_C);
  Serial.println("GetCWakeSampleRate");
  Serial.println(value, HEX);      
  value &= 0b00001111;
  return (MC3672_cwake_sr_t) (value);
}

//Get the range control
MC3672_range_t MC3672::GetRangeCtrl(void)
{
  /* Read the data format register to preserve bits */
  uint8_t value;
  value = readRegister8(MC3672_REG_RANGE_C);
  Serial.println("GetRangeCtrl");
  Serial.println(value, HEX);
  value &= 0x70;
  return (MC3672_range_t) (value >> 4);
}

//Get the range control
MC3672_resolution_t MC3672::GetResolutionCtrl(void)
{
  /* Read the data format register to preserve bits */
  uint8_t value;
  value = readRegister8(MC3672_REG_RANGE_C);
  Serial.println("GetResolutionCtrl");
  Serial.println(value, HEX);  
  value &= 0x07;
  return (MC3672_resolution_t) (value);
}

//Initialize the MC3672 sensor and set as the default configuration
bool MC3672::start(void)
{
  
#ifdef MC3672_CFG_BUS_I2C
  Wire.begin(); // Initialize I2C
#else
  digitalWrite(chipSelectPin, HIGH); //Set active-low CS low to start the SPI cycle 
  pinMode(chipSelectPin, OUTPUT);   
  SPI.setDataMode (SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.begin(); // Initialize SPI
#endif  
   
  //Init Reset
  reset();
  SetMode(MC3672_MODE_STANDBY);

  //SetWakeAGAIN
  SetWakeAGAIN(MC3672_GAIN_1X);
  //SetSniffAGAIN
  SetSniffAGAIN(MC3672_GAIN_1X);  

  /* Check I2C connection */
  uint8_t id = readRegister8(MC3672_REG_PROD);
  if (id != 0x71)
  {
    /* No MC3672 detected ... return false */
    Serial.println("No MC3672 detected!");
    Serial.println(id, HEX);
    return false;
  }
  SetRangeCtrl(MC3672_RANGE_8G);        //Range: 8g
  SetResolutionCtrl(MC3672_RESOLUTION_14BIT);     //Resolution: 14bit
  SetCWakeSampleRate(MC3672_CWAKE_SR_DEFAULT_54Hz);   //Sampling Rate: 50Hz
  SetMode(MC3672_MODE_CWAKE);         //Mode: Active
    
  delay(50);    
    
  return true;
}

void MC3672::stop()
{
  SetMode(MC3672_MODE_SLEEP); //Set mode as Sleep
}

//Read the raw counts and SI units mearsurement data
MC3672_acc_t MC3672::readRawAccel(void)
{
  float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f}; //{2g, 4g, 8g, 16g, 12g}
  float faResolution[6] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f}; //{6bit, 7bit, 8bit, 10bit, 12bit, 14bit}

  byte rawData[6];
  readRegisters(MC3672_REG_XOUT_LSB, rawData, 6);  // Read the six raw data registers into data array
  x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
  y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
  z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

  AccRaw.XAxis = (short) (x);
  AccRaw.YAxis = (short) (y);
  AccRaw.ZAxis = (short) (z);
  AccRaw.XAxis_g = (float) (x) / faResolution[CfgResolution]*faRange[CfgRange];
  AccRaw.YAxis_g = (float) (y) / faResolution[CfgResolution]*faRange[CfgRange];
  AccRaw.ZAxis_g = (float) (z) / faResolution[CfgResolution]*faRange[CfgRange];

  return AccRaw;
}
