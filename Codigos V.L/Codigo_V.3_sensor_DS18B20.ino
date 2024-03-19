/*
Version 3.0 Codigo Sensor DB18B20
-----IMPORTANTE------
PARA PODER EJECUTAR EL CODIGO EN EL ARDUINO GIGA R1 
ES NECESARIO ACTUALIZAR LA LIBRERIA OneWire.h ESTA SE TIENE QUE 
DESCARGAR DESDE SU REPOSITORIO DE GITHUB COPIARLA Y REMPLAZARLA EN LA
CARPETA DE LIBRERIAS DE ARDUINO DE NUESTRA LAPTOP DE TRABAJO.
Conexion
DT  -> D2
VCC -> 5V
GND -> GND
*/
 
#include <OneWire.h>
#include <DallasTemperature.h>
 
OneWire ourWire(2); // se establece la conexion al pin 2
 
DallasTemperature sensors(&ourWire);
 
void setup(){
  delay(1000);
  Serial.begin(9600);
  sensors.begin();//inicia sensor
}
void loop(){
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
 
  Serial.print("temperatura = ");
  Serial.print(temp);
  Serial.println("°C");
  delay(500);
}
