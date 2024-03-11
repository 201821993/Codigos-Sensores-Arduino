//SONRES CONECTADOS HASTA EL MOMENTO 
/*
CONEXION A INTERNET
BMP280
Sensor TDS
Pluviometro
particulas suspendidas
rayos ultravioleta

hoy sensor 
veleta y anenometro
mq135 y mq131

*/

/*Incio configuraciones de router y del servidor  */
//recuerda conectar la antena del arduino giga
#include <ArduinoHttpClient.h>
#include <WiFi.h>
// Configuración de la red WiFi
char ssid[] = "netis_70F8D0";
char password[] = "carlos4522";

// Configuración del servidor
const char* serverAddress = "192.168.166.122"; // ip (modifcar por la ip de la computadora, checar con el comando "ipconfig")
const int serverPort = 3001;  // Puerto para la conexión HTTP

/*Fin configuraciones de router y el servidor */

// Incio sensor BMP280

/*
VCC del sensor a 3.3v del Arduino
GND del sensor a GND del Arduino
SCL del sensor a SCL21 del Arduino
SDA del sensor a SDA20 del Arduino
*/
    #include <Wire.h>
    #include <SPI.h>
    #include <Adafruit_BMP280.h>
    
    #define BMP_SCK  (13)
    #define BMP_MISO (12)
    #define BMP_MOSI (11)
    #define BMP_CS   (10)
    
    Adafruit_BMP280 bmp; // I2C

// fin sensor BMP280

//Inicio Sensor tds 
    #define TdsSensorPin A2
    #define VREF 3.3 //  o 5.0   analog reference voltage(Volt) of the ADC
    #define SCOUNT 30 // sum of sample point
    int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
    int analogBufferTemp[SCOUNT];
    int analogBufferIndex = 0,copyIndex = 0;
    float averageVoltage = 0,tdsValue = 0,temperature = 25;
// fin sensor tds


// Inicio codigo pluviometro 
    const int Sensor = 3; // Salida digital 3 del arduino giga
    const int umbralTiempo = 300;
    volatile int ISRContador = 0;
    int contador = 0;
    float litros = 0;
    long tiempoContador = 0;
    long tiempoHora = 0;
    String dataPluviometro = "";
// fin codigo pluviometro

//Incio codigo de PMS7003 , sensor de particulas suspendidas
#include <PMS.h>
struct pms5003data {
 uint16_t framelen;
 uint16_t pm10_standard, pm25_standard, pm100_standard;
 uint16_t pm10_env, pm25_env, pm100_env;
 uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um,
particles_100um;
 uint16_t unused;
 uint16_t checksum;
};
struct pms5003data data;
// Fin  codigo de PMS7003 , sensor de particulas suspendidas


// Inicio uv sensor
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
// fin uv sensor

// wifi 
String postData = "";



void setup() {
  // INICIO SENSOR BMP280
     Serial.begin(9600);
      while ( !Serial ) delay(100);   // wait for native usb
      Serial.println(F("BMP280 test"));
      unsigned status;
      //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
      status = bmp.begin(0x76);
      if (!status) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                          "try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
      }
    
      /* Default settings from datasheet. */
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
// FIN SENSOR BMP280

// Inicio sensor tds
     Serial.begin(115200);
     pinMode(TdsSensorPin,INPUT);
// Fin sensor tds

// Inicio codigo pluviometro
      pinMode(Sensor,INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(Sensor),contadorLitros,RISING);
// fin del codigo del pluviometro

// Inicio conexion al wifi
 WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a la red WiFi...");
  }

  Serial.println("Conectado a la red WiFi");
  // Fin conexion al wifi


//Incio codigo de PMS7003 , sensor de particulas suspendidas
   Serial2.begin(9600);
// Fin  codigo de PMS7003 , sensor de particulas suspendidas

// inicio uv sensor 
pinMode(UVOUT, INPUT);
pinMode(REF_3V3, INPUT);
Serial.println("ML8511 example");
// fin uv sensor
}// llave del void setup()

void loop() {
  // INICIO BMP280
          Serial.print(F("Temperature = "));
          Serial.print(bmp.readTemperature());
          Serial.println(" *C");
      
          Serial.print(F("Pressure = "));
          Serial.print(bmp.readPressure());
          Serial.println(" Pa");
      
          Serial.print(F("Approx altitude = "));
          Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
          Serial.println(" m");
      
          Serial.println();
    
  // FIN SENSOR BMP280

  // Inicio sensor tds
         static unsigned long analogSampleTimepoint = millis();
       if(millis()-analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
       {
       analogSampleTimepoint = millis();
       analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
       analogBufferIndex++;
       if(analogBufferIndex == SCOUNT)
       analogBufferIndex = 0;
       }
       static unsigned long printTimepoint = millis();
       if(millis()-printTimepoint > 800U)
       {
       printTimepoint = millis();
       for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
       analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
       averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
       float compensationCoefficient=1.0+0.02*(temperature-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
       float compensationVolatge=averageVoltage/compensationCoefficient; //temperature compensation
       tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
       //Serial.print("voltage:");
       //Serial.print(averageVoltage,2);
       //Serial.print("V ");
       Serial.print("TDS Value:");
       Serial.print(tdsValue,0); // esta variable se deve de mandar
       Serial.println("ppm ");
       Serial.println(ISRContador);
       }
 // Fin sensor tds


// inicio pluviometro 
      if(millis()<tiempoHora){
      tiempoHora = 0;
     }
    if(millis()-tiempoHora>60000){ // millis()-tiempoHora>3600000UL
      tiempoHora = millis();
      litros = ISRContador * 0.27945; //0.011   y viento de 1.492 MPH
      Serial.println("Litros por m2 caidos en una Hora (L/m2");
      Serial.println(litros);
      Serial.print("");
      //envioServidor(litros);
      ISRContador = 0;
      contador = 0;
      }
// fin pluviometro 







// Inicio codigo de PMS7003 , sensor de particulas suspendidas
 if (readPMSdata(&Serial2)) {
 // reading data was successful!
 Serial.println();
 Serial.println("---------------------------------------");
// Serial.println("Concentration Units (standard)");
// Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
// Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
// Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
// Serial.println("---------------------------------------");
 Serial.println("Concentration Units (environmental)");
 Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
 Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
 Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
          // Serial.println("---------------------------------------");
          // Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
          // Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
          // Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
          // Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
          // Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
          // Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
          // Serial.println("---------------------------------------");
 }
// fin codigo de PMS7003 , sensor de particulas suspendidas

//inicio sensor uv

  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);

  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;

  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level

//  Serial.print("output: ");
//  Serial.print(refLevel);
//
//  Serial.print("ML8511 output: ");
//  Serial.print(uvLevel);
//
//  Serial.print(" / ML8511 voltage: ");
//  Serial.print(outputVoltage);

  Serial.print(" / UV Intensity (mW/cm^2): ");
  Serial.print(uvIntensity);

  Serial.println();

// fin sensor uv


//INICIO ENVIO DE DATOS AL SERVIDOR
 // Datos que deseas enviar
  postData = "tds=String(tdsValue)&temperatura=String(bmp.readTemperature())&pressure=String(bmp.readTemperature())&uv=String(uvIntesity)&pm10_env=String(data.pm10_env)&pm25=String(data.pm25_env)";
  sendingDataToServer();
  

//FIN ENVIO DE  DATOS  AL SERVIDOR


  
} // llave del void loop ()

// metod tds | inicio
int getMedianNum(int bArray[], int iFilterLen)
{
     int bTab[iFilterLen];
     for (byte i = 0; i<iFilterLen; i++) bTab[i] = bArray[i];
     int i, j, bTemp;
     for (j = 0; j < iFilterLen - 1; j++)
     {
     for (i = 0; i < iFilterLen - j - 1; i++)
     {
     if (bTab[i] > bTab[i + 1])
     {
     bTemp = bTab[i];
     bTab[i] = bTab[i + 1];
     bTab[i + 1] = bTemp;
     }
     }
     }
     if ((iFilterLen & 1) > 0)
     bTemp = bTab[(iFilterLen - 1) / 2];
     else
     bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
     return bTemp;
} 
// metodo tds | fin

// metodo de pluviometro
void contadorLitros(){
  
    if(millis()>(tiempoContador+umbralTiempo)){
        ISRContador++;
        tiempoContador = millis();
       }
 }
// Fin metodo pluviometro 


// Inicio codigo de PMS7003 , sensor de particulas suspendidas

boolean readPMSdata(Stream *s) {
 if (! s->available()) {
 return false;
 }
 // Read a byte at a time until we get to the special '0x42' start-byte
 if (s->peek() != 0x42) {
 s->read();
 return false;
 }
 // Now read all 32 bytes
 if (s->available() < 32) {
 return false;
 }

 uint8_t buffer[32];
 uint16_t sum = 0;
 s->readBytes(buffer, 32);
 // get checksum ready
 for (uint8_t i=0; i<30; i++) {
 sum += buffer[i];
 }
 /* debugging
 for (uint8_t i=2; i<32; i++) {
 Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
 }
 Serial.println();
 */
 // The data comes in endian'd, this solves it so it works on all platforms
 uint16_t buffer_u16[15];
 for (uint8_t i=0; i<15; i++) {
 buffer_u16[i] = buffer[2 + i*2 + 1];
 buffer_u16[i] += (buffer[2 + i*2] << 8);
 }
 // put it into a nice struct
 memcpy((void *)&data, (void *)buffer_u16, 30);
 if (sum != data.checksum) {
 Serial.println("Checksum failure");
 return false;
 }
 // success!
 return true;
}

// Fin codigo de PMS7003 , sensor de particulas suspendidas  (metodo)

// sensor uv
//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// fin sensor uv

// metodo wifi
void sendingDataToServer(){

    detachInterrupt(digitalPinToInterrupt(Sensor));
    // detachInterrupt(digitalPinToInterrupt(SensorPin));

  
  // Crea una instancia de HttpClient
  WiFiClient wifi;
  HttpClient http = HttpClient(wifi, serverAddress, serverPort);

  // Construye la URL completa
  String url = "http://192.168.166.122/api/showData";  // No es necesario especificar la dirección completa si ya la especificaste en la configuración del cliente

  // Realiza la solicitud POST
  http.post(url, "application/x-www-form-urlencoded", postData);

  // Obtiene y muestra la respuesta del servidor
  String response = http.responseBody();
  Serial.print("Respuesta del servidor: ");
  Serial.println(response);

  // Cierra la conexión
  http.stop();
  
       attachInterrupt(digitalPinToInterrupt(Sensor),contadorLitros,RISING);

  // attachInterrupt(digitalPinToInterrupt(SensorPin),countup,RISING);



  
  }
