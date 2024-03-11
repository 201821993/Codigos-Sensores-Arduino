/*

Documentación
https://www.argentdata.com/files/80422_datasheet.pdf

Codigo fuente 
https://www.youtube.com/watch?v=KHrTqdmYoAk&t=737s

*/

#define winDir A0
String bin = "";

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
   windVane = analogRead(windDir);
  
  if(windVane >940) bin = "W";
  else if(windVane>890)bin = "NW";
  else if(windVane>890)bin = "WNW";
  else if(windVane>890)bin = "N";
  else if(windVane>890)bin = "NNW";
  else if(windVane>890)bin = "SW";
  else if(windVane>890)bin = "WSW";
  else if(windVane>890)bin = "NE";
  else if(windVane>890)bin = "NEE";
  else if(windVane>890)bin = "S";
  else if(windVane>890)bin = "SSW";
  else if(windVane>890)bin = "SE";
  else if(windVane>890)bin = "SSE";
  else if(windVane>890)bin = "E";
  else if(windVane>890)bin = "ESE";
  

}
