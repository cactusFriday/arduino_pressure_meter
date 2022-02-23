#include "GyverStepper.h"
#include <Wire.h>
#include "hsc_ssc_i2c.h"


// define working area parameters
#define RADIUS        1000      //радиус окружности
#define Z_STEP        100       //величина шага по Z
#define X_STEP        10        //величина шага по Х

#define EPS              10     // Для определения необходимости явного измерения конца хорды:
// В конце прохода по хорде, последний шаг неравномерен и попадает на расстояние
// (X_STEP .. 0) от правого конца хорды.
// Явное измерение проводится только если расстояние до конца хорды БОЛЬШЕ, чем
// X_STEP деленный на EPS.
// При EPS=10 и X_STEP=100, сравниваемое расстояние = 100 / 10 = 10
#define PRESS_MEAS_ERROR 40     // На сколько нужно уменьшить "нулевое" давление
// !Есть ошибка: при нулевом давлении показывает больше нуля,
// Мы видели, что 40 Па, поэтому отнимаем 40.

// define sensor parameters
#define MEAS_DELAY      150         //Сколько необходимо ждать перед замером
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

// *define stepper parameters*
#define step_speed 1000              //скорость
#define step_accel 0                 //ускорение

// Задержка на переключение STEP пина у драйвера 
// (для стабильности больше чем дефолтная у A4988, а у него 1 мкс)
// one step is 1.8 degree -> 200 steps per full rotate
GStepper< STEPPER2WIRE> stepperX(200, STEPX_PIN, DIRX_PIN);
GStepper< STEPPER2WIRE> stepperZ(200, STEPZ_PIN, DIRZ_PIN);

// прототипы функций
void move_stepperX(int16_t x);
void move_stepperZ(int16_t z);
void measure_env();
void move_measuring(byte m_num);
void refresh_xyz();
void print_table();
void print_current_position();
void control();

// глобальная структура для координат
struct Bup {
  int x;
  int z;
};

Bup Bkup;

String pressure;        // переменная для давления
String pressure_m;
String temp;            // переменная для температуры
int num_exp = 0;        // номер эксперимента

int HEIGHT = 2 * RADIUS;  //высота измеримой области
int WIDTH = 2 * RADIUS;   //ширина измеримой области

unsigned long timer;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // ждём открытия порта
  while (!Serial) {}
  // печатаем сообщение об успешной инициализации Serial-порта
  Serial.println("Serial init OK");

  Bkup.x = 0;
  Bkup.z = 0;

  stepperX.setAcceleration(step_accel);
  stepperX.setSpeed(step_speed);
  stepperX.reset();

  stepperZ.setAcceleration(step_accel);
  stepperZ.setSpeed(step_speed);
  stepperZ.reset();
  delay(200);
}

void loop() {
  /* Entry point для Ардуины. */

  control();
}


void move_stepperX(int16_t x) {
  /* Перемещает датчик с помощью движения двигателя (по оси Х)
  !: по абсолютным координатам */

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


void move_stepperZ(int16_t z) {
  /* Перемещает датчик с помощью движения двигателя (по оси Y)
  !: по абсолютным координатам */

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
  /* Алгоритм измерений. 
  Изначально перемещается в верхнюю точку окружности,
  после, в цикле, перемещается на левый край хорды окружности,
  двигаясь вправо - проводит измерения,
  перемещается на следующую хорду в левый конец.*/

  int z_steps = (int)(HEIGHT / X_STEP);

  stepperX.reset();
  stepperZ.reset();
  refresh_xyz();
  Serial.println("Приступаем к измерениям...");
  Serial.print("[N],\t[X],\t[Z],\t[P],\t[P_m],\t[T]\n");  //шапка таблицы

  move_stepperZ(RADIUS);
  measure_env();
  print_table();

  for (int i = 0; i < z_steps; i++) {
    move_stepperZ(stepperZ.getCurrent() - Z_STEP);
    int chord_l = sqrt(sqr(RADIUS) - sqr(stepperZ.getCurrent()));
    move_stepperX(-chord_l);
    move_chord(chord_l);
  }

  move_stepperZ(-RADIUS);
  move_stepperX(0);
  measure_env();
  print_table();
}


void move_chord(int16_t chord_l) {
  /* Двигается по оси Х по длине хорды и проводит измерения по шагам */

  // Первое измерение проводится отдельно
  measure_env();
  print_table();

  int16_t full_chord = 2 * chord_l;
  int16_t steps = (full_chord / X_STEP);

  // В цикле едем в правый конец хорды и измеряем
  for (int i = 0; i < steps; i++) {
    move_stepperX(stepperX.getCurrent() + X_STEP);
    measure_env();
    print_table();
  }

  // если в конце измерений датчик слишком далеко от конца хорды ->
  // явно измеряем конец хорды
  int last_step_eps = (int)(X_STEP / EPS);
  if (chord_l - stepperX.getCurrent() > last_step_eps) {
    move_stepperX(chord_l);
    measure_env();
    print_table();
  }
}


void refresh_xyz() {
  /* Обновляет значения координат в глобальной структуре */

  Bkup.x = stepperX.getCurrent();
  Bkup.z = stepperZ.getCurrent();
}


void measure_env() {
  /* Проводит измерения температуры и давления */

  delay(MEAS_DELAY);

  byte i = 0;
  float temp_p = 0.0, tot_p = 0.0;
  float temp_t = 0.0, tot_t = 0.0;
  float max_p = 0.0, min_p = 9999.0;
  byte final_i = 0;
  struct cs_raw ps;
  float p, t;
  char p_str[10];

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

  // Записываем результат в глобальные переменные
  temp_p = (tot_p / (float)final_i) - (float)PRESS_MEAS_ERROR;
  pressure = temp_p;
  temp = tot_t / (float)final_i;
  pressure_m = (max_p - min_p) / temp_p;
}


void print_table() {
    /* Выводит значения измерения на экран через запятую в формате:
    measure_num,  X,  Z,  P,  P_m,  T
    measure_num - порядковый номер проведенного измерения,
    X, Z - координаты измерения,
    P, T - давление и температура,
    P_m - усредненное давление (max-min/2)*/

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
  Serial.print("\n");
}


void print_current_position() {
  Serial.print("Current X position: ");
  Serial.println(stepperX.getCurrent());
  Serial.print("Current Z position: ");
  Serial.println(stepperZ.getCurrent());
  Serial.println("----------------------------------------\n");
}


void control() {
  /* Функция поездки по вводу символов, после ввода 'm' начинаются измерения */

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
      print_current_position();
      str = "";
    }
    else if (int(str[0]) == 114) { //r - reset
      stepperZ.reset();
      stepperX.reset();
      Serial.println("Сброс двигателей");
      print_current_position();
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
