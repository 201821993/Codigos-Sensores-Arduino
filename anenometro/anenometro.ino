// https://mechatronics-books.blogspot.com/p/estacion-meteorologica-con-arduino.html
const int SENSOR=3;

const int RecordTime = 3; //Definir el tiempo de medición (segundos)

const int SensorPin = 3;  //Definir pin de interrupción (2 o 3 @ Arduino Uno)

int InterruptCounter;

float WindSpeed;

void setup()

{

  pinMode(SensorPin, INPUT_PULLUP);

  Serial.begin(9600);

}

void loop() {

  meassure();

  Serial.print("Wind Speed: ");

  Serial.print(WindSpeed);       //Velocidad en km/h

  Serial.print(" km/h - ");

  Serial.print(WindSpeed / 3.6); //Velocidad en m/s

  Serial.println(" m/s");

}

void meassure() {

  InterruptCounter = 0;

  attachInterrupt(digitalPinToInterrupt(SensorPin), countup, RISING);

  delay(1000 * RecordTime);

  detachInterrupt(digitalPinToInterrupt(SensorPin));

  WindSpeed = (float)InterruptCounter / (float)RecordTime * 2.4;

}

void countup() {

  InterruptCounter++;

}
