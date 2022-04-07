
#define pin_OUT 3
const byte interruptPin = 2;
unsigned int rot = 0;
unsigned long int tm;
unsigned long int rpm = 0;
unsigned long int rpmF = 0;
unsigned int dt = 0;
double PWM =0 ;
int period = 500;//dt
float set_RPM = 100.0;
float kp =152.0;
float ki = 2.0 ;
float kd = 30.0;


int PID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}




// бегущее среднее с адаптивным коэффициентом
float expRunningAverageAdaptive(float newVal) {
  static float filVal = 0;
  float k;
  // резкость фильтра зависит от модуля разности значений
  if (abs(newVal - filVal) > 1.5) k = 1.2;
  else k = 0.03;
  
  filVal += (newVal - filVal) * k;
  return filVal;
}









const int NUM_READ = 10;  // количество усреднений для средних арифм. фильтров
float midArifm2(float newVal) {
  static byte counter = 0;     // счётчик
  static float prevResult = 0; // хранит предыдущее готовое значение
  static float sum = 0;  // сумма
  sum += newVal;   // суммируем новое значение
  counter++;       // счётчик++
  if (counter == NUM_READ) {      // достигли кол-ва измерений
    prevResult = sum / NUM_READ;  // считаем среднее
    sum = 0;                      // обнуляем сумму
    counter = 0;                  // сброс счётчика
  }
  return prevResult;
}


void detect() {
    rot++; // прибавляем единичку к счётчику обротов
    dt = millis() - tm; // вычисляем время с последнего расчёта
    if( dt >= 100 ){ // если прошло 100мс или более, то начинаем расчёт
        rpm = rot*60000/dt/1000;
        rot = 0; // обнуляем счётчик
        tm = millis(); // запоминаем время расчёта
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(pin_OUT,OUTPUT);
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), detect, RISING);

    tm = millis();
}

void loop() {

      rpmF = expRunningAverageAdaptive(rpm);
     PWM = PID(rpmF, set_RPM, kp, ki, kd, period/100,50,255);
     
    analogWrite(pin_OUT,PWM);
    Serial.println(rpmF);

  
    
}
