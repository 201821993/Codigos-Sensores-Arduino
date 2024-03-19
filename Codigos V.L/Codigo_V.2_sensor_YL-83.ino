int analogValue; //0 - 1023
bool digitalValue;
/*
* Codigo version 1 YL-83 modulo de lluvia
* pulsos de 500 a 0 lluvia
* pulsos de 501 a 1023 no llueve
* Conexiones 
* VCC- 5v
* GND-GND
* DO - PIN 4
* AO - A0
*/
 
void setup(){
  Serial.begin(9600);
}
 
void loop(){
  //entrada analogica
  analogValue = analogRead(0);
  if(analogValue < 300)
  Serial.println("LLueve");
  else if (analogValue <500)
  Serial.println("Brisea");
  else
  Serial.println("NO LLueve");
  Serial.println("analogValue"+String(analogValue));
 
  //Entrada digital pin9 arduino
  digitalValue = digitalRead(9);
  if(digitalValue == HIGH)
    Serial .println(">>>NO LLUEVE");
  else
    Serial.println("<<<LLUEVE");
  delay(2000);
}
