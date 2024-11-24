/* #include <Servo.h>

Servo servoX;  // Servo para eje X (pan)
Servo servoY;  // Servo para eje Y (tilt)

int posX = 90;  // Posición inicial del servo X
int posY = 90;  // Posición inicial del servo Y

void setup() {
  Serial.begin(9600);
  servoX.attach(11);  // Servo X en pin 11
  servoY.attach(9);   // Servo Y en pin 9
  servoX.write(posX);
  servoY.write(posY);
  Serial.println("Sistema listo");
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');  // Lee la línea completa
    Serial.println("Datos crudos: " + data);

    // Parsear los datos de "X<angleX>Y<angleY>"
    int xIndex = data.indexOf('X');
    int yIndex = data.indexOf('Y');

    if (xIndex != -1 && yIndex != -1) {
      int angleX = data.substring(xIndex + 1, yIndex).toInt();
      int angleY = data.substring(yIndex + 1).toInt();
      
      Serial.print("Ángulo X recibido: ");
      Serial.println(angleX);
      Serial.print("Ángulo Y recibido: ");
      Serial.println(angleY);

      // Limita los ángulos para asegurar que estén en el rango de 0 a 180 grados
      posX = constrain(angleX, 0, 180);
      posY = constrain(angleY, 0, 180);

      // Mueve los servos a las posiciones ajustadas
      servoX.write(posX);
      servoY.write(posY);

      Serial.print("Servo X movido a: ");
      Serial.println(posX);
      Serial.print("Servo Y movido a: ");
      Serial.println(posY);
    } else {
      Serial.println("Comando no válido");
    }
  }
}
 */


/* #include <Servo.h>

Servo servoX, servoY;

void setup() {
  servoX.attach(9); // Pin para el servo X
  servoY.attach(11); // Pin para el servo Y
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n'); // Leer hasta el fin de línea
    int separator = data.indexOf(',');          // Separar valores con coma
    if (separator > 0) {
      int posX = data.substring(0, separator).toInt();    // Obtener posición X
      int posY = data.substring(separator + 1).toInt();  // Obtener posición Y
      servoX.write(posX); // Mover servo X
      servoY.write(posY); // Mover servo Y
    }
  }
} */


#include <Servo.h>

Servo servoX;
Servo servoY;

void setup() {
  Serial.begin(9600);
  servoX.attach(9);  // Pin al que está conectado el servo X
  servoY.attach(10); // Pin al que está conectado el servo Y
  servoY.write(60);
  servoX.write(140);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      int xPos = data.substring(0, commaIndex).toInt();
      int yPos = data.substring(commaIndex + 1).toInt();
      servoX.write(xPos); // Mover servo X
      servoY.write(yPos); // Mover servo Y
    }
  }
}
