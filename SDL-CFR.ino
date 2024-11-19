// SEGUIDOR DE LINEA - CFR

// Pines de los sensores QTR-8RC
const int sensorPins[6] = {13, 14, 27, 26, 25, 32};  // Son 6 los empleados

//Posiciones de los sensores
const int positionSensor[6] = {-2, -1, 0, 0, 1, 2};
int sensorsNumbers = 0;

// Variables para almacenar los tiempos de lectura
int sensorValues[6];

// Pines de salida
const int pwmLeft = 5;  // Pin PWM para el motor 1
const int V1 = 16;    // Pin de control del motor 1 (alimentación)
const int GND1 = 17;    // Pin de control del motor 1 (tierra)
const int pwmRight = 15;  // Pin PWM para el motor 2
const int V2 = 2;    // Pin de control del motor 2 (alimentación)
const int GND2 = 4;    // Pin de control del motor 2 (tierra)

// Variables PWM
const int pwmLeftChannel = 0;   // Canal PWM para el motor 1
const int pwmRightChannel = 1;   // Canal PWM para el motor 2
const int frequency = 5000; // Frecuencia de PWM en Hz
const int resolution = 8;   // Resolución de 8 bits (0-255)

// Variables del algorimo PID
float Kp = 10 ; //Control propocional
float Ki = 0.01; //Control integral
float Kd = 2; //Control derivado
float P;
float I;
float D;
float error;
float lastError = 0;
float position;
float correction;

// Velocidades
const int defaultVelocity = 100; // Velocidad minima establecida, de 0 a 255
int leftVelocity;
int rightVelocity;
bool enableMotors = true;


void setup() {
  Serial.begin(115200);
  
  // Configura los pines de los sensores como entradas
  for (int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT_PULLDOWN); // Los valores de lectura son 0 o 1
  }

  // Configura los pines de control de los motores
  pinMode(V1, OUTPUT);
  pinMode(GND1, OUTPUT);
  pinMode(V2, OUTPUT);
  pinMode(GND2, OUTPUT);
  digitalWrite(V1, HIGH);
  digitalWrite(GND1, LOW);
  digitalWrite(V2, HIGH);
  digitalWrite(GND2, LOW);
  

  // Configura los pines de control del PWM
  ledcAttachChannel(pwmLeft, frequency, resolution, pwmLeftChannel);
  ledcAttachChannel(pwmRight, frequency, resolution, pwmRightChannel);
}


void loop() {
  readSensors();
  pidController();
  motorsVelocity();
}


void readSensors(){
  // Lee las mediciones de cada sensor
  for (int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  // Imprime los valores en el monitor serial (solo para visualizacion de funcionamiento)
  Serial.print("Valores de los sensores: ");
  for (int i = 0; i < 6; i++) {
      sensorValues[i] = digitalRead(sensorPins[i]);
  }
  
  for (int i = 0; i < 6; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
}


void pidController(){
  // Se reestablecen los valores iniciales con cada loop
  position = 0;
  sensorsNumbers = 0;
  sensorsNumbers = 0;

  // Se determina el número de sensores activados
  for(int i = 0; i < 6; i++) {
    if(sensorValues[i] == 1){
      position += positionSensor[i];
      ++sensorsNumbers;
    }
  }

  // Se pondera el total de los sensores activados (hasta 3 a la vez) para obtener la ubicacion
  if(sensorsNumbers >= 1){
    if(position != 0) {
      position = position / sensorsNumbers;
    } else {
      position = 0;
    }
  }

  // Caso en el que no se detecte posición debido a que ningun sensor esta sobre una linea
  if(position == 0 && sensorsNumbers == 0){
    enableMotors = false;
  } else {
    enableMotors = true;
  }

  // Impresión de los valores de la posicion
  Serial.print("Posicion: ");
  Serial.print(position);
  Serial.println();

  // Calculo de los parametros
  error = position - 0;
  P = error;
  I += error;
  D = error - lastError;
  correction = (Kp * P) + (Ki * I) + (Kd * D);
  lastError = error;
}
  

void motorsVelocity(){
  // Magnitud de las velocidades
  leftVelocity = defaultVelocity - correction;
  rightVelocity = defaultVelocity + correction;

  // Delimitacion de las velocidades maximas y minimas (0 a 255) ya que no se aplica la funcion de retroceder
  if(leftVelocity > 255){
    leftVelocity = 255;
  }
  if(leftVelocity < 0){
    leftVelocity = 0;
  }
  if(rightVelocity > 255){
    rightVelocity = 255;
  }
  if(rightVelocity < 0){
    rightVelocity = 0;
  }

  // Activación de los motores
  if(enableMotors == true) {
    ledcWrite(pwmLeft, leftVelocity);
    ledcWrite(pwmRight, rightVelocity);
  } else {
    ledcWrite(pwmLeft, 0);
    ledcWrite(pwmRight, 0);
  }
}
