    const int Sensor = 3; // Salida digital 3 del arduino giga
    const int umbralTiempo = 300;
    volatile int ISRContador = 0;
    int contador = 0;
    float litros = 0;
    long tiempoContador = 0;
    long tiempoHora = 0;
    String dataPluviometro = "";
    
void setup() {
  // put your setup code here, to run once:
   pinMode(Sensor,INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(Sensor),contadorLitros,RISING);

}

void loop() {
  // put your main code here, to run repeatedly:
        if(millis()<tiempoHora){
      tiempoHora = 0;
     }
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

}
