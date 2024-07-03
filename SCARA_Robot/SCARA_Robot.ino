/*
  Robot SCARA Arduino UNO - FIUNA - Robotica I
  Adolfo Monje - Magno Caballero - Brenda Cruz
  
    Este codigo recibe los puntos de la trayectoria que debe seguir la herramienta del brazo
  y la abertura para el objeto. Almacena los datos y los ejecuta ciclicamente pasando por 
  los puntos dados.
  Limites articulares:
  # art 1: -88 a 268        ()
  # art 2: 0 a 220          ()
  # art 3: -157 a 157       (desde art 2 > 227 es -67 a 67)
  # art 4: -165 a 165       ()
  # gripper: 0 a 100        Punto de agarre por debajo de art 2
*/
#include <AccelStepper.h> // AccelStepper: http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
#include <MultiStepper.h>
#include <Servo.h>
#include <math.h> // ?

#define limitSwitch1 9
#define limitSwitch2 A3
#define limitSwitch3 11
#define limitSwitch4 10

// (Type:driver, STEP, DIR)
AccelStepper stepper1(1, 2, 5); // Primera articulacion (R - Base)
AccelStepper stepper2(1, 3, 6); // Segunda articulacion (P - Eje Z)
AccelStepper stepper3(1, 4, 7); // Tercera articulacion (R)
AccelStepper stepper4(1, 12, 13); // Cuarta articulacion (R - Herramienta) 
MultiStepper steppers;

Servo gripperServo;  // Servo de la Pinza

String content = "";
int data[7];

const float theta1AngleToSteps = 44.444444; 
const float theta2AngleToSteps = 35.555555;
const float phiAngleToSteps = 10;
const float zDistanceToSteps = 100;

int theta1Array[100]; // 100 son los Puntos que puede recibir por serial
int theta2Array[100];
int phiArray[100];
int zArray[100];
int gripperArray[100];
int positionsCounter = 0; // cantidad de puntos recibidos

void setup() {
  Serial.begin(115200);

  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);

  // Stepper motors max speed // run() esta atado a esto, mas no runSpeed()
  stepper1.setMaxSpeed(1000); // Las velocidades por encima de 1000 pasos por segundo no son fiables según documentación, pero depende del clk del controlador
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(2000);

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);

  gripperServo.attach(A0, 600, 2500); // Rango de pulsos admitidos por el servo en microsegundos
  data[6] = 110; // Valor de apertura del gripper, 110 es apertura maxima y 180 minima
  gripperServo.write(data[6]);
  delay(1000);
  // Declaraciones temporales
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(-5000);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(-900);
  // while (!Serial.available()){
    
  // }
  // delay(100);
  // if (Serial.available()){
  //   if (Serial.readString() == "2,0,0,0,0,0,0"){
  //     homing();
  //   }else{
  //     stepper1.setCurrentPosition(0);
  //     stepper2.setCurrentPosition(-5000);
  //     stepper3.setCurrentPosition(0);
  //     stepper4.setCurrentPosition(-900);
  //   }
  // }
  //serialFlush();
}

void loop() {
  
  if (Serial.available()) {
    content = Serial.readString(); // Read the incomding data
    // Extraccion de datos del String y ubicarlos en variables int (data[] array)
    for (int i = 0; i < 7; i++) {
      int index = content.indexOf(","); // Localizar la primera ","
      data[i] = atol(content.substring(0, index).c_str()); // Extract the number from start to the ","
      content = content.substring(index + 1); // Remover el numero del String
    }
    /*
     data[0] - SAVE button status - 1: guardar data. - 2: reestablecer todo.
     data[1] - RUN button status - 0 o 1.
     data[2] - Joint 1 angle
     data[3] - Joint 2 angle
     data[4] - Joint 3 angle
     data[5] - Z position
     data[6] - Gripper value */
    
    // guardado de los datos
    if (data[0] == 1) { // se convierte a pasos los angulos dados
      theta1Array[positionsCounter] = data[2] * theta1AngleToSteps; //store the values in STEPS = angles * angleToSteps variable
      theta2Array[positionsCounter] = data[3] * theta2AngleToSteps;
      phiArray[positionsCounter] = data[4] * phiAngleToSteps;
      zArray[positionsCounter] = data[5] * zDistanceToSteps;
      gripperArray[positionsCounter] = data[6];
      positionsCounter++;
      Serial.println(); // Aviso para el control de que se puede recibir la siguiente trama
    } else if (data[0] == 2) { // clear data // Reestablece los puntos articulares en cero y va a la posicion de origen
      // Clear the array data to 0
      memset(theta1Array, 0, sizeof(theta1Array));
      memset(theta2Array, 0, sizeof(theta2Array));
      memset(phiArray, 0, sizeof(phiArray));
      memset(zArray, 0, sizeof(zArray));
      memset(gripperArray, 0, sizeof(gripperArray));
      positionsCounter = 0;
      data[0] = 0;
      data[1] = 0;
      gripperServo.write(110);
      
      long positions[4] = {0, -5000, 0, -900}; // Posicion de origen al reestablecer los valores
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(-5000);
      stepper3.setCurrentPosition(0);
      stepper4.setCurrentPosition(-900);
      serialFlush(); // Elimina todos los datos que pudiesen haber en el buffer del serial
    }
    
    
  }

  // If RUN button is pressed
  if (data[1] == 1) {
    Serial.println("RUN");
    // data[1] = 0; // impide que se vuelva a hacer el movimiento
    // Ejecucion de pasos guardados
    for (int i = 0; i < positionsCounter; i++) {
      Serial.println(i);
      if (zArray[i] != 0){
        long positions[4] = {theta1Array[i], zArray[i], theta2Array[i], phiArray[i]};
        steppers.moveTo(positions);
        steppers.runSpeedToPosition();
        // Chequeo de cambios en el programa
        if (Serial.available()) {
            data[1] = 0; // para que no se vuelva a ejecutar
            break; // parada
        }
        if (i == 0) { // agarrar / soltar
          delay(500); // Tiempo para eliminar la inercia del movimiento antes de agarrar el objeto
          gripperServo.write(gripperArray[i]);
          delay(800); // Tiempo para que agarre el objeto
        }else if (gripperArray[i] != gripperArray[i - 1]) { // cuando la posicion sea diferente
          delay(500); // Tiempo para eliminar la inercia del movimiento antes de soltar el objeto
          gripperServo.write(gripperArray[i]); // actualizar acción
          delay(800); // esperar 0.8s para que el servo agarre o suelte
        }
      }
    }
    //Serial.println("Posicion final");
    // Ejecucion de pasos guardados en reversa
    for (int i = positionsCounter - 1; i >= 0; i--) { // Regresa a la posicion de donde se agarro el objeto inicialmente
      //Serial.println(i);
      if (data[1] == 0){
        break;
      }
      if (zArray[i] != 0){
        if (i == positionsCounter - 1) { // agarrar / soltar
          delay(500); // Tiempo para eliminar la inercia del movimiento antes de agarrar el objeto
          gripperServo.write(gripperArray[i]);
          delay(800); // Tiempo para que agarre el objeto
        }
        long positions[4] = {theta1Array[i], zArray[i], theta2Array[i], phiArray[i]};
        steppers.moveTo(positions);
        steppers.runSpeedToPosition();
        // Chequeo de cambios en el programa
        if (Serial.available()) {
            data[1] = 0; // para que no se vuelva a ejecutar
            break; // parada
        }
        
      }
    }
    //Serial.println("Posicion inicial");
  }
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is > 0
    Serial.read();         // get one character
  }
}

void homing() { 
  int conteoPasos = 0;

  stepper4.setSpeed(-500);
  stepper3.setSpeed(-500);
  stepper2.setSpeed(500);
  stepper1.setSpeed(-500);
  while (digitalRead(limitSwitch4) != 1 || digitalRead(limitSwitch3) != 1 || digitalRead(limitSwitch2) != 1 || digitalRead(limitSwitch1) != 1){
    if (digitalRead(limitSwitch4) != 1){
      
      stepper4.runSpeed();
    }else{
      Serial.println("switch4");
    }
    if (digitalRead(limitSwitch3) != 1){
      Serial.println("switch3");
      stepper3.runSpeed();
    }
    if (digitalRead(limitSwitch2) != 1){
      Serial.println("switch2");
      stepper2.runSpeed();
    }
    if (digitalRead(limitSwitch1) != 1){
      Serial.println("switch1");
      stepper1.runSpeed();
    }
  }
  stepper4.setCurrentPosition(-1650);
  stepper3.setCurrentPosition(-5590);
  stepper2.setCurrentPosition(0);
  stepper1.setCurrentPosition(-3950);
  long positions[4] = {0, -5000, 0, -900}; // Posicion de origen
      steppers.moveTo(positions);
      steppers.runSpeedToPosition();
}

/////////// Antes de usar Multistepper
// stepper1.moveTo(theta1Array[i]); // moveTo() recalcula la velocidad, por lo que se puede llamar despues a setSpeed() (segun ref...)
// stepper2.moveTo(zArray[i]);  
// stepper3.moveTo(theta2Array[i]);
// stepper4.moveTo(phiArray[i]);
// stepper1.setSpeed(data[7]); // ultimos valores recibidos de data[7 y 8]
// stepper2.setSpeed(data[7]);
// stepper3.setSpeed(data[7]);
// stepper4.setSpeed(data[7]);
// // Este while se ejecuta tantas veces sea necesario por cada iteracion de i
// while (stepper1.currentPosition() != theta1Array[i] || stepper2.currentPosition() != zArray[i] || stepper3.currentPosition() != theta2Array[i] || stepper4.currentPosition() != phiArray[i]) {
//   stepper1.run(); // se ejecuta si queda pendiente un paso de acuerdo a moveTo(...)
//   stepper2.run();
//   stepper3.run();
//   stepper4.run();
// }
