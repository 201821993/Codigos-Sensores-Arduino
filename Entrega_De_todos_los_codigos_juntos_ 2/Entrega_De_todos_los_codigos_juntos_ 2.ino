//SONRES CONECTADOS HASTA EL MOMENTO 
/*
CONEXION A INTERNET
BMP280
Sensor TDS
Pluviometro
particulas suspendidas
rayos ultravioleta
veleta y anenometro
mq131

falta mq131

*/

/*Incio configuraciones de router y del servidor  */
//recuerda conectar la antena del arduino giga
#include <ArduinoHttpClient.h>
#include <WiFi.h>
// Configuración de la red WiFi
char ssid[] = "netis_70F8D0";
char password[] = "carlos4522";

// Configuración del servidor
const char* serverAddress = "192.168.166.43"; // ip (modifcar por la ip de la computadora, checar con el comando "ipconfig")
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

// Inicio MQ135  

#define mq135Pin A10  // Pin analógico para MQ-135
float  mq135Value;  // Variables para almacenar las lecturas de los sensores
float  ppm135;  // Variables para almacenar las concentraciones en ppm
// FIN MQ135

// Inicio mq31
#include "MQ131.h" 
#define mq131Pin A11  // Pin analógico para MQ-135

// fin mq131

// inicio anemometro 
    // Codigo anenometro
    static unsigned long RecordTime ;    
    const int SensorPinAnemometro = 7; // checar que el arduino giga 
    int InterruptCounter;
    float WindSpeed;
    // fin codigo anenometro 

// fin anemometro

// inicio veleta 
 #define windDir A8 // pin tied to Vout for voltage divider circuit
 int incoming = 0;

// fin veleta

// Inicio variables del programa 
String postData = ""; // cadena que se usa para enviar datos al servidor 
String postDataLluvia = ""; // cadena que se usa para enviar datos al servidor (exclusivo para la lluvia)
float temperatura = 0;
float presion = 0;
float altitud = 0;
float pm1_0 = 0; // Guarda el valor de pm 1.0: 
float pm2_5 = 0; // Guarda el valor de Pm 2.5
float pm10 = 0; // Guarda el valor de  Pm 10 
float mq131ug_m3  = 0; 
static unsigned long flowExecution ;
int  tiempoEsperaGrupoA = 60000;
int tdsLectura = 90000;
int tdscalculo = 91000; 
int tiempoEsperaPSM7003 = 180000;  // variable para controlar las particulas suspendidas (3:30 3 minutos con 30 segundos)
int tiempoEsperaM131 = 210000; // variable para ccontrolar el mq135  cada 3: (3 minutos con 30 segundos )
int tiempoEsperaAneno = 280000 ; // variable que controla el anenometro, el codigo del anenometro se ejecuta cada 4 minutos con un tiempo de extension de 16 segundos 
int tiempoEnvioServidor = 310000 ;
int tiempoLecturaAnenometro = 999999999;
int cierreLecturaAnenometro = 296000 ; // apartir de 4 minutos con 16 segundos
 float uvIntensity = 0;
// Fin variables del sisetma


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


// Inicio mq131

  MQ131.begin( 2, A5, LOW_CONCENTRATION, 1000000);  

  Serial.println("Calibration in progress...");
  
  MQ131.calibrate();
  
  Serial.println("Calibration done!");
  Serial.print("R0 = ");
  Serial.print(MQ131.getR0());
  Serial.println(" Ohms");
  Serial.print("Time to heat = ");
  Serial.print(MQ131.getTimeToRead());
  Serial.println(" s");

// Fin mq131
 flowExecution = millis();
//Inicio anenometro

pinMode(SensorPinAnemometro,INPUT_PULLUP);
//attachInterrupt(digitalPinToInterrupt(SensorPinAnemometro),countup,RISING);
 
// fin anenometro



}// llave del void setup()

void loop() {
  
  // INICIO Grupo A 

          if(millis()-flowExecution > tiempoEsperaGrupoA )
       {

        tiempoEsperaGrupoA = 99999999;
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
          temperatura = bmp.readTemperature();
        presion =   bmp.readPressure();
         altitud =  bmp.readAltitude(1013.25);
        // FIN SENSOR BMP280


         
        //inicio sensor uv
        
          int uvLevel = averageAnalogRead(UVOUT);
          int refLevel = averageAnalogRead(REF_3V3);
        
          //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
          float outputVoltage = 3.3 / refLevel * uvLevel;
        
           uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level
        
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
         
       }
        
     // fin grupo A
    
  
 // grupo b

  // Inicio sensor tds
        // static unsigned long analogSampleTimepoint = millis();
       if(millis()- flowExecution > tdsLectura) //every 40 milliseconds,read the analog value from the ADC
       {
       //analogSampleTimepoint = millis();  verificar si ya no se usan para borrarlas
       tdsLectura = 999999999;
       analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
       Serial.println(" Now time for good bye "+String(analogBuffer[analogBufferIndex]));
       analogBufferIndex++;
       if(analogBufferIndex == SCOUNT)
       analogBufferIndex = 0;
       }
     //  static unsigned long printTimepoint = millis();
       if(millis()- flowExecution > tdscalculo)
       {
       // printTimepoint = millis(); verificar si ya no se usan para borrarlas
       tdscalculo = 99999999;
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
//      if(millis()<tiempoHora){
//      tiempoHora = 0;
//     }
    if(millis()-tiempoHora > 3600000){ // millis()-tiempoHora>3600000UL
      tiempoHora = millis();
      litros = ISRContador * 0.27945; //0.011   y viento de 1.492 MPH
      Serial.println("Litros por m2 caidos en una Hora (L/m2");
      Serial.println(litros);
      Serial.print("");
      //envioServidor(litros);
      ISRContador = 0;
      contador = 0;
      sendingDataToServer();

      }
// fin pluviometro 







// Inicio codigo de PMS7003 , sensor de particulas suspendidas
  if(millis()-flowExecution > tiempoEsperaPSM7003     )
       {
         if (readPMSdata(&Serial2)) {
          tiempoEsperaPSM7003= 99999999;
          
                 // reading data was successful!
                 Serial.println();
                 //Serial.println("---------------------------------------");
                // Serial.println("Concentration Units (standard)");
                // Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
                // Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
                // Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
                // Serial.println("---------------------------------------");
                 Serial.println("Concentration Units (environmental)");
                 Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
                 Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
                 Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
                 pm1_0 = data.pm10_env;
                 pm2_5 = data.pm25_env;
                 pm10 = data.pm100_env;
                          // Serial.println("---------------------------------------");
                          // Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
                          // Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
                          // Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
                          // Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
                          // Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
                          // Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
                          // Serial.println("---------------------------------------");
         }
}
// fin codigo de PMS7003 , sensor de particulas suspendidas






// inicio mq131 sensor ozono
if(millis()-flowExecution > tiempoEsperaM131  && millis()-flowExecution <= 270000  ){
tiempoEsperaM131 = 99999999;
Serial.println("Sampling...");
  MQ131.sample();
//  Serial.print("Concentration O3 : ");
//  Serial.print(MQ131.getO3(PPM));
//  Serial.println(" ppm");
//  Serial.print("Concentration O3 : ");
//  Serial.print(MQ131.getO3(PPB));
//  Serial.println(" ppb");
//  Serial.print("Concentration O3 : ");
//  Serial.print(MQ131.getO3(MG_M3));
//  Serial.println(" mg/m3");
  Serial.print("Concentration O3 : ");
  Serial.print(MQ131.getO3(UG_M3));
  Serial.println(" ug/m3");
  mq131ug_m3 = MQ131.getO3(UG_M3);
}

// fin mq131 sensor ozono
 
// Inicio Anenometro 
// este codigo se ejecutara despues de 4 minutos con un tiempo de extension de 16 de segundos
    if(millis()-flowExecution > tiempoEsperaAneno   ){
      RecordTime = millis();
      tiempoLecturaAnenometro = 3000 ;
      tiempoEsperaAneno = 999999999;
      attachInterrupt(digitalPinToInterrupt(SensorPinAnemometro),countup,RISING);       
    }
          if(millis() - RecordTime > tiempoLecturaAnenometro ){
                  Serial.println("Interrupt counter    ->   "+String(InterruptCounter) );
                  detachInterrupt(digitalPinToInterrupt(SensorPinAnemometro));
                  WindSpeed = ((float)InterruptCounter / (float) 3)  * 2.4;
                  Serial.println(WindSpeed);
                  attachInterrupt(digitalPinToInterrupt(SensorPinAnemometro),countup,RISING); 
                   RecordTime = millis();
                  InterruptCounter = 0;
                  incoming = analogRead(windDir);
                  Serial.println("Direccion del viento "+String(incoming));
                 
           }
          if( millis()- flowExecution > cierreLecturaAnenometro ){
                 detachInterrupt(digitalPinToInterrupt(SensorPinAnemometro));
                 tiempoEsperaAneno = 999999999;
                 tiempoLecturaAnenometro = 9999999999 ;
                 cierreLecturaAnenometro = 9999999999;
                
                 
          }

           



//Fin Anenometro 

//INICIO ENVIO DE DATOS AL SERVIDOR
  
      if(millis()-flowExecution > tiempoEnvioServidor  ){     

       // Datos que deseas enviar
          postData = "&temperatura="+String(temperatura)+"&pressure="+String(presion)+"&altitud="+String(altitud)+"&ozono="+String(mq131ug_m3)+"&air_quality="+String(23)+"&uv="+String(uvIntensity)+"&pm10_env="+String(pm1_0)+"&pm25="+String(pm2_5)+"&pm100_env="+String(pm10)+"&windSpeed="+String(WindSpeed)+"&direction="+String(incoming);
          sendingDataToServer();
      // "tds="+String(tdsValue)+
      }

 

  

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
    // condicion que evalua si tdsvalue es mayor a cero, significa que esta lluviendo y hay agua
    if(millis()>(tiempoContador+umbralTiempo) and tdsValue >0 ){
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
  // antes de mandar los datos regresamos , el tiempo de espera del anenometro se aumenta, el de flowExcetucion se regresa a cero.


    detachInterrupt(digitalPinToInterrupt(Sensor));
  

  
  // Crea una instancia de HttpClient
  WiFiClient wifi;
  HttpClient http = HttpClient(wifi, serverAddress, serverPort);

  // Construye la URL completa
  String url = "http://192.168.166.43/api/insert-data";  // No es necesario especificar la dirección completa si ya la especificaste en la configuración del cliente

  // Realiza la solicitud POST
  http.post(url, "application/x-www-form-urlencoded", postData);

  // Obtiene y muestra la respuesta del servidor
  String response = http.responseBody();
  Serial.print("Respuesta del servidor: ");
  Serial.println(response);

  // Cierra la conexión
  http.stop();
  
  attachInterrupt(digitalPinToInterrupt(Sensor),contadorLitros,RISING);
 // attachInterrupt(digitalPinToInterrupt(SensorPinAnemometro),countup,RISING);

// al final todas variables regresan a sus valores originales 
  flowExecution = millis();
  tiempoEsperaGrupoA = 60000;
  tdsLectura = 90000;
  tdscalculo = 91000; 
  tiempoEsperaPSM7003 = 180000; 
  tiempoEsperaM131 = 210000; // 
  tiempoEsperaAneno = 280000 ;  
  tiempoEnvioServidor = 310000 ;
  cierreLecturaAnenometro = 296000 ;
  tiempoLecturaAnenometro = 9999999999 ;
                 
                 
  }
  // fin metodo wifi

  
  //Inicio anenometro
  void countup(){
   InterruptCounter++;
}
  
  // fin anenometro
  
