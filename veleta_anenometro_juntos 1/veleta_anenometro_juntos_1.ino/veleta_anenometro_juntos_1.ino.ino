
//#include <HttpClient.h>
////codigo veleta
//#include <WiFi.h>
//char ssid[] = "netis_70F8D0"; // Totalplay-04A6 CCAI_TESE
//char password[] = "carlos4523"; //  04A6DBCFCHH7muMQwifi  TESE_ccai22
//const char* serverUrl = "http://192.168.166.103:3001/api/dataRealTime"; // URL del servidor Node donde se guarda los datos del pluviometro.


#define windDir A13 // pin tied to Vout for voltage divider circuit

// These arrays are specific to the ADS wind vane with a 10kOhm fixed resistor in the voltage divider
int sensorExp[] = {66,84,93,126,184,244,287,406,461,599,630,702,785,827,886,945};
float dirDeg[] = {112.5,67.5,90,157.5,135,202.5,180,22.5,45,247.5,225,337.5,0,292.5,315,270};
char* dirCard[] = { "SE","SE","SSW","S","NNE","NE","WSW","SW","NNW","N","WNW","NW","W"};

int sensorMin[] = {63,80,89,120,175,232,273,385,438,569,613,667,746,812,869,931};
int sensorMax[] = {69,88,98,133,194,257,301,426,484,612,661,737,811,868,930,993};

int incoming = 0;
float angle = 0;

//fin codigo veleta

//codidgo pluviometro
const int Sensor =2;
const int umbralTiempo = 300;
volatile int ISRContador = 0;
//int contador = 0;
float litros = 0;
long tiempoContador = 0;
long tiempoHora = 0;
String dataPluviometro = "";
// fin codigo pluviometro


 // Codigo anenometro
static unsigned long RecordTime = millis();    
const int SensorPinAnemometro= 3;
int InterruptCounter;
float WindSpeed;
// fin codigo anenometro  


void setup() {

//   // Conectamos o wifi
// WiFi.begin(ssid, password);
//
//// Enquanto não conectar printamos um "."
//  while(WiFi.status() != WL_CONNECTED)
//   {
//     Serial.print(".");
//       delay(1000);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
//    }
      // fin codigo para hacer la conexión wife



  // codig opluviometro
   Serial.begin(9600);
   pinMode(Sensor,INPUT_PULLUP);
     
      attachInterrupt(digitalPinToInterrupt(Sensor),contadorLitros,RISING);
      // fin del codigo del pluviometro
 
      
pinMode(SensorPinAnemometro,INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(SensorPinAnemometro),countup,RISING); 

     // codigo anenomitros

}
void contadorLitros(){

     
  if(millis()>(tiempoContador+umbralTiempo)){
    ISRContador++;
    tiempoContador = millis();
    }
 }

void loop() {
  Serial.println("");

// codigo anenometro
//  meassure();

if(millis() - RecordTime > 3000 ){

  Serial.println("Interrupt counter    ->   "+String(InterruptCounter) );
  detachInterrupt(digitalPinToInterrupt(SensorPinAnemometro));
  WindSpeed = ((float)InterruptCounter / (float) 3)  * 2.4;
  Serial.println(WindSpeed);
  RecordTime = millis();
  attachInterrupt(digitalPinToInterrupt(SensorPinAnemometro),countup,RISING); 
  InterruptCounter = 0;
  delay(500);
       }


incoming = analogRead(windDir);
  for(int i=0; i<=15; i++) {
   if(incoming >= sensorMin[i] && incoming <= sensorMax[i]) {
    angle = dirDeg[i];
    break;
   }
  }
 
  Serial.print("EL angulo del viento "+String(angle));
  Serial.print(":");
       
//  

//  
// // delay(1500);
   //fin codigo veleta


  // codigo pluviometro

  if(millis()<tiempoHora){
      tiempoHora = 0;
     }
    if(millis()-tiempoHora>180000000){ // millis()-tiempoHora>3600000UL
      tiempoHora = millis();
      litros = ISRContador * 0.27945; //0.011   y viento de 1.492 MPH
      Serial.println("Litros por m2 caidos en una Hora (L/m2");
      Serial.println(litros);
      Serial.print("");
      ISRContador = 0;
      }



//   HttpClient http;
//  http.begin(serverUrl);
//  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
//   dataPluviometro = "litrosm2="+String(litros); // Ejemplo de datos, puedes ajustar esto según tus necesidades
//
//  int httpResponseCode = http.POST(dataPluviometro);
//  // Comprobar la respuesta del servidor
//  if (httpResponseCode > 0) {
//    String response = http.getString();
//    Serial.println("Respuesta del servidor: " + response);
//  } else {
//    Serial.print("Error en la solicitud. Código de respuesta HTTP: ");
//    Serial.println(httpResponseCode);
//  }
//  



//String outputArduino = "["+String(incoming)+"]["+String(litros,2)+"]["+String(WindSpeed,2)+"]";
//Serial.println(outputArduino);


  
//   Serial.println("sÍ ENTRE PERO VAMOS A VER QUE PEDO ************** "+String(InterruptCounter));
//     WindSpeed = (float)InterruptCounter / 3  *2.4;
//  Serial.println(WindSpeed);
//delay(100);
}
// codigo anenometro 

void meassure(){
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(SensorPinAnemometro),countup,RISING);
  delay(1000 * RecordTime);
  detachInterrupt(digitalPinToInterrupt(SensorPinAnemometro));
 WindSpeed = (float)InterruptCounter / (float)RecordTime  *2.4;
  Serial.println(WindSpeed);
  
  }
 void countup(){
   InterruptCounter++;
   //Serial.println("  ************** "+String(InterruptCounter));
 }
   
 // fin codigo anenometro   
