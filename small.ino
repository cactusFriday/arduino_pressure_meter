#include "GyverStepper.h"
#include <Wire.h>
#include "hsc_ssc_i2c.h"
#include <TroykaI2CHub.h>

// define working area parameters
#define GAP_L         11500     //длина маленькой щели
#define DEPTH         5000      //высота лопаток
#define CENTER_NUMS   5         //количество шагов по Z по центру
#define CENTER_X_NUMS 10        //количество шагов по Х по центру
#define EDGE_STRIDE   100       //малый шаг у края по Z
#define EDGE_NUMS     15        //количество шагов по Z у края
#define EDGE_X_NUMS   50        //количество шагов по Х у края

// define sensor parameters
#define MEAS_DELAY      150         //Сколько необходимо ждать перед замером
#define CH0             0           //Канал, полученный со сканера для основного датчика
#define CH1             1           //Канал, полученный со сканера для датчика в трубе
#define SLAVE_ADDR      0x28        //Адрес датчика
#define OUTPUT_MIN      1638.4      //10 % от 2^14
#define OUTPUT_MAX      14745.4     //90 % от 2^14
#define PRESSURE_MIN    -1600       // min is 0 for sensors that give absolute values
#define PRESSURE_MAX    1600        // results in pascals)

// *define pins for steppers*
// X stepper
#define STEPX_PIN   12
#define DIRX_PIN    13
// Z stepper
#define STEPZ_PIN   8
#define DIRZ_PIN    7
// common enable pin
#define EN_PIN      12

// *define stepper parameters*
#define step_speed 1000              //скорость
#define step_accel 0              //ускорение

// Задержка на переключение STEP пина у драйвера (для стабильности больше чем дефолтная у A4988, а у него 1 мкс)
// one step is 1.8 degree -> 200 steps per full rotate
GStepper< STEPPER2WIRE> stepperX(200, STEPX_PIN, DIRX_PIN);
GStepper< STEPPER2WIRE> stepperZ(200, STEPZ_PIN, DIRZ_PIN);

TroykaI2CHub splitter;

// прототипы функций
void move_stepperX(int16_t x);
void move_stepperZ(int16_t z);
void measure();
void measure_env();
void move_measuring(byte m_num);
void refresh_xyz();
void print_table();
void control();

// глобальные переменные для координат
struct Bup {
  int x;
  int z;
};

Bup Bkup;

String pressure;        // переменная для давления
String pressure_m;
String pressure_m_env;
String temp;            // переменная для температуры
String p_env;
String t_env;
int num_exp = 0;        // номер эксперимента

int stride;             //расстояние шага по X
unsigned long timer;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // ждём открытия порта
  while (!Serial) {}
  // печатаем сообщение об успешной инициализации Serial-порта
  Serial.println("Serial init OK");
  // начало работы с I²C хабом
  splitter.begin();
  Serial.println("Splitter init OK");
  delay(100);
  // Устанавливаем изначальную шину на основной датчик
  splitter.setBusChannel(CH0);

  Bkup.x = 0;
  Bkup.z = 0;
  stepperX.setAcceleration(step_accel);
  stepperX.setSpeed(step_speed);
  stepperX.reset();

  stepperZ.setAcceleration(step_accel);
  stepperZ.setSpeed(step_speed);
  stepperZ.reset();
  delay(2000);
}

void loop() {
  control();
}

// Функция для перемещения одного мотора Х
void move_stepperX(int16_t x) {
  stepperX.setSpeed(step_speed);
  stepperX.setRunMode(FOLLOW_POS);
  stepperX.setTarget(x, ABSOLUTE);
  while (true)
  {
    if (!stepperX.tick()) {
      break;
    }
    refresh_xyz();
  }
}
// Функция для перемещения одного мотора Z
void move_stepperZ(int16_t z) {
  stepperZ.setSpeed(step_speed);
  stepperZ.setRunMode(FOLLOW_POS);
  stepperZ.setTarget(z, ABSOLUTE);
  while (true)
  {
    if (!stepperZ.tick()) {
      break;
    }
    refresh_xyz();
  }
}

void move_measuring() {
  stepperX.reset();
  stepperZ.reset();
  refresh_xyz();
  Serial.println("Приступаем к измерениям...");
  Serial.print("[N],\t[X],\t[Z],\t[P],\t[P_m],\t[T],\t[P_env],\t[P_env_m],\t[T_env]\n");  //шапка таблицы
  // измерения одной канавки туда и обратно
  boolean isNear = false;
  int dir = -1;
  //****************************************Переходим к измерению маленькой щели****************************************
  num_exp = 1;
  stride = GAP_L / CENTER_X_NUMS;                            //размер шага по Х
  int z_edge = EDGE_STRIDE * EDGE_NUMS;                      //высота измельченной области
  int z_center = DEPTH - 2 * z_edge;                         //высота центральной области
  int x_edge_stride = GAP_L / EDGE_X_NUMS;                   //размер шага у краев по Х
  int z_center_stride = z_center / CENTER_NUMS;                  //размер шага в центральной области по Z
  //  верхний слой мельченый
  for (byte i = 0; i < EDGE_NUMS; i++) {
    dir *= -1;
    for (byte j = 0; j < EDGE_X_NUMS; j++) {
      measure_env(0);
      measure_env(1);
      print_table();
      num_exp++;
      move_stepperX(Bkup.x + (dir * x_edge_stride));
    }
    measure_env(0);
    measure_env(1);
    print_table();
    num_exp++;
    move_stepperZ(Bkup.z + EDGE_STRIDE);
  }
  //Средний слой с большими шагом
  for (byte i = 0; i < CENTER_NUMS; i++) {
    dir *= -1;
    for (byte j = 0; j < CENTER_X_NUMS; j++) {
      measure_env(0);
      measure_env(1);
      print_table();
      num_exp++;
      move_stepperX(Bkup.x + (dir * stride));
    }
    measure_env(0);
    measure_env(1);
    print_table();
    num_exp++;
    move_stepperZ(Bkup.z + z_center_stride);
  }
  //  последний нижний измельченный слой
  for (byte i = 0; i < EDGE_NUMS; i++) {
    dir *= -1;
    for (byte j = 0; j < EDGE_X_NUMS; j++) {
      measure_env(0);
      measure_env(1);
      print_table();
      num_exp++;
      move_stepperX(Bkup.x + (dir * x_edge_stride));
    }
    measure_env(0);
    measure_env(1);
    print_table();
    num_exp++;
    move_stepperZ(Bkup.z + EDGE_STRIDE);
  }
}

void refresh_xyz() {
  Bkup.x = stepperX.getCurrent();
  Bkup.z = stepperZ.getCurrent();
}

// Функция для измерения параметров вторым датчиком
void measure_env(byte num) {
  delay(MEAS_DELAY);
  byte i = 0;
  float temp_p = 0.0, tot_p = 0.0;
  float temp_t = 0.0, tot_t = 0.0;
  float max_p = 0.0, min_p = 9999.0;
  byte final_i = 0;
  struct cs_raw ps;
  float p, t;
  char p_str[10];
  // Меняем шину I2C на второй датчик
  if (num == 1) {
    splitter.setBusChannel(CH1);
  }
  // Проводим 10 измерений, чтобы потом взять среднее
  for (byte i = 0; i < 10; i++) {
    // вызов функции для получения данных с датчика и занесения их в структуру ps
    int16_t el = ps_get_raw(SLAVE_ADDR, &ps);
    // функция перевода необработанного давления в Паскали
    ps_convert(ps, &p, &t, OUTPUT_MIN, OUTPUT_MAX, PRESSURE_MIN, PRESSURE_MAX);
    temp_p = p;
    temp_t = t;
    if (temp_p < 2000.0 && temp_t < 5000.0) {
      max_p = (temp_p > max_p ? temp_p : max_p);
      min_p = (temp_p < min_p ? temp_p : min_p);
      tot_p += temp_p;
      tot_t += temp_t;
      final_i++;
    } else {
    }
  }
  // Меняем шину I2C на основной датчик
  if (num == 1) {
    float temp_p = (tot_p / (float)final_i) - 40.0;
    p_env = temp_p;
    t_env = tot_t / (float)final_i;
    pressure_m_env = (max_p - min_p) / temp_p;
    splitter.setBusChannel(CH0);
  } else {
    float temp_p = (tot_p / (float)final_i) - 40.0;
    pressure = temp_p;
    temp = tot_t / (float)final_i;
    pressure_m = (max_p - min_p) / temp_p;
  }
}

void print_table() {
  Serial.print(num_exp);
  Serial.print(",\t");
  Serial.print(-stepperX.getCurrent());
  Serial.print(",\t");
  Serial.print(stepperZ.getCurrent());
  Serial.print(",\t");
  Serial.print(pressure);
  Serial.print(",\t");
  Serial.print(pressure_m);
  Serial.print(",\t");
  Serial.print(temp);
  Serial.print(",\t");
  Serial.print(p_env);
  Serial.print(",\t");
  Serial.print(pressure_m_env);
  Serial.print(",\t");
  Serial.print(t_env);
  Serial.print("\n");
}

// Функция поездки по вводу символов, после ввода 'm' начинаются измерения
void control() {
  String str = "";
  Serial.println("INITIALIZED\n Настройка начальной позиции.\n Управление:\n\
                  's'\tперемещение вниз\n\
                  'w'\tперемещение вверх\n\
                  'd'\tперемещение вправо\n\
                  'a'\tперемещение влево\n\
                  'p'\tостановка\n\
                  'm'\tпереход к режиму измерений\n\
                  'r'\tсброс двигателей\n");
  while (1) {
    if (Serial.available() > 0) {
      delay(20);
      char c = Serial.read();
      str += c;
    }
    if (int(str[0]) == 119) { //w
      stepperX.brake();
      stepperZ.setRunMode(KEEP_SPEED);
      stepperZ.setSpeed(-step_speed);
      str = "";
      Serial.println("вверх");
      Serial.println("----------------------------------------\n");
      while (true) {
        stepperZ.tick();
        if (Serial.available() > 0) {
          break;
        }
      }
    }
    if (int(str[0]) == 115) { //s
      stepperX.brake();
      stepperZ.setRunMode(KEEP_SPEED);
      stepperZ.setSpeed(step_speed);
      str = "";
      //      _timer = millis();
      Serial.println("вниз");
      Serial.println("----------------------------------------\n");
      while (true) {
        stepperZ.tick();
        if (Serial.available() > 0) {
          break;
        }
      }
    }
    if (int(str[0]) == 100) { //d
      stepperZ.brake();
      stepperX.setRunMode(KEEP_SPEED);
      stepperX.setSpeed(step_speed);
      str = "";
      //      _timer = millis();
      Serial.println("вправо");
      Serial.println("----------------------------------------\n");
      while (true) {
        stepperX.tick();
        if (Serial.available() > 0) {
          break;
        }
      }
    }
    if (int(str[0]) == 97) { //a
      stepperZ.brake();
      stepperX.setRunMode(KEEP_SPEED);
      stepperX.setSpeed(-step_speed);
      str = "";
      Serial.println("влево");
      Serial.println("----------------------------------------\n");
      while (true) {
        stepperX.tick();
        if (Serial.available() > 0) {
          break;
        }
      }
    }
    else if (int(str[0]) == 112) { //p
      stepperZ.brake();
      stepperX.brake();
      Serial.println("остановка");
      Serial.print("Current X position: ");
      Serial.println(stepperX.getCurrent());
      Serial.print("Current Z position: ");
      Serial.println(stepperZ.getCurrent());
      Serial.println("----------------------------------------\n");
      str = "";
    }
    else if (int(str[0]) == 114) { //r - reset
      stepperZ.reset();
      stepperX.reset();
      Serial.println("Сброс двигателей");
      Serial.print("Current X position: ");
      Serial.println(stepperX.getCurrent());
      Serial.print("Current Z position: ");
      Serial.println(stepperZ.getCurrent());
      Serial.println("----------------------------------------\n");
      str = "";
    }
    else if (int(str[0]) == 109) { //m - measure
      stepperZ.brake();
      stepperX.brake();
      Serial.println("Переходим к алгоритму измерений");
      move_measuring();
      break;
    }
    else {
      str = "";
    }
  }
}
