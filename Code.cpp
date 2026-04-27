// =====================================================
// CONFIGURATION
// =====================================================

constexpr uint8_t T1_VOLTAGE = A0;
constexpr uint8_t T2_VOLTAGE = A1;
constexpr uint8_t T3_VOLTAGE = A3;

constexpr float VREF = 5.0f;
constexpr uint16_t ADC_MAX = 1023;

constexpr uint32_t SAMPLE_PERIOD_MS  = 10;    // 100 Hz
constexpr uint32_t CONTROL_PERIOD_MS = 100;   // 10 Hz
constexpr float CONTROL_TS = CONTROL_PERIOD_MS / 1000.0f;

constexpr uint8_t FILTER_WINDOW = 10;

constexpr float Rw1 = 9949.71f;
constexpr float Rw2 = 9845.01f;
constexpr float Rw3 = 9873.95f;

constexpr float G1 = 1.092699f;
constexpr float G2 = 1.98946f;
constexpr float G3 = 2.005565f;

constexpr float Vref = 2.4986f;

constexpr float a_th = 0.003354016434680530000f;
constexpr float b_th = 0.000256523550896126000f;
constexpr float c_th = 0.000002605970120720520f;
constexpr float d_th = 0.000000063292612648746f;

constexpr float R01 = 9531.714843202744872981f;
constexpr float R02 = 9572.7855972823f;
constexpr float R03 = 9581.15367753648f;

constexpr float y4 = -0.0106122449f;
constexpr float y3 = 0.191755102f;
constexpr float y2 = -0.5639591837f;
constexpr float y1 = 0.3685714286f;

// =====================================================
// REGULATEUR DISCRET
// =====================================================

struct PIDFBankEntry {
  uint8_t slotId;
  float kp;
  float ki;
  float kd;
  float tf;
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
};

constexpr uint8_t PIDF_BANK_SIZE = 4;
PIDFBankEntry pidfBank[PIDF_BANK_SIZE] = {
  {1, 4.57879f, 0.035f, 0.4f, 0.05f, 8.58054f, -12.57704f, 4.0f, -1.0f, 0.0f},
  {2, 4.57879f, 0.035f, 0.4f, 0.05f, 8.58054f, -12.57704f, 4.0f, -1.0f, 0.0f},
  {3, 4.57879f, 0.035f, 0.4f, 0.05f, 8.58054f, -12.57704f, 4.0f, -1.0f, 0.0f},
  {4, 4.57879f, 0.035f, 0.4f, 0.05f, 8.58054f, -12.57704f, 4.0f, -1.0f, 0.0f}
};

uint8_t activePidfIndex = 0;

float u0 = 0.0f, u1 = 0.0f, u2 = 0.0f;
float e0 = 0.0f, e1 = 0.0f, e2 = 0.0f;

constexpr float UMAX = 95.0f;
constexpr float UMIN = -95.0f;

// =====================================================
// MODES DE COMMANDE
// =====================================================

float consigne = 25.0f;
bool setpointEnabled = false;
bool openLoopEnabled = false;
int openLoopPWM = 0;
float transitionStartT3 = 25.0f;

// =====================================================
// TEMPERATURE AMBIANTE
// =====================================================

float Tamb = 23.5f;

// =====================================================
// MODELE T3 ESTIMEE
// =====================================================

float t3m_d1 = -0.00362247f;
float t3m_d2 =  0.00379366f;

float t3m_c1 = -1.97070244f;
float t3m_c2 =  0.97070244f;

float x0 = 0.0f, x1 = 0.0f, x2 = 0.0f;
float T3est0 = 0.0f, T3est1 = 0.0f, T3est2 = 0.0f;

float uop = 0.0f;

// =====================================================
// PWM
// =====================================================

const int PWM1 = 11;
const int PWM2 = 12;

// =====================================================
// TEMPS
// =====================================================

unsigned long lastSampleTime = 0;
unsigned long lastControlTime = 0;
unsigned long t0 = 0;

// =====================================================
// BUFFER SERIE NON BLOQUANT
// =====================================================

static const size_t RX_BUFFER_SIZE = 512;
char rxBuffer[RX_BUFFER_SIZE];
size_t rxIndex = 0;

// =====================================================
// FILTRE MOYENNE GLISSANTE
// =====================================================

float t1Buffer[FILTER_WINDOW] = {0};
float t2Buffer[FILTER_WINDOW] = {0};
float t3Buffer[FILTER_WINDOW] = {0};

float t1Sum = 0.0f;
float t2Sum = 0.0f;
float t3Sum = 0.0f;

uint8_t filterIndex = 0;
uint8_t filterCount = 0;

float T1_filt = 23.5f;
float T2_filt = 23.5f;
float T3_filt = 23.5f;

// =====================================================
// PROTOTYPES
// =====================================================

void handleSerial();
void processCommand(char *line);

float computeTemperature(float R, float R0, float a, float b, float c, float d);
void acquireAndFilterTemperatures();
void updateT3Estimate();

void applyControl(float u);
void stopControl();
void resetControllerState();

uint8_t selectPidfIndexForTransition(float startT3, float newSetpoint);
float computePIDDiscreteAntiWindup(float e);

void sendDataLine(float t_s, float T1, float T2, float T3, float T3_est, float e, float u);

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);

  t0 = millis();
  lastSampleTime = millis();
  lastControlTime = millis();

  T1_filt = Tamb;
  T2_filt = Tamb;
  T3_filt = Tamb;

  T3est0 = Tamb;
  T3est1 = Tamb;
  T3est2 = Tamb;

  Serial.println("INFO,Arduino ready");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  handleSerial();

  unsigned long now = millis();

  while (now - lastSampleTime >= SAMPLE_PERIOD_MS) {
    lastSampleTime += SAMPLE_PERIOD_MS;
    acquireAndFilterTemperatures();
  }

  while (now - lastControlTime >= CONTROL_PERIOD_MS) {
    lastControlTime += CONTROL_PERIOD_MS;

    float t_s = (lastControlTime - t0) / 1000.0f;

    updateT3Estimate();

    if (setpointEnabled) {
      e0 = consigne - T3est0;
      u0 = computePIDDiscreteAntiWindup(e0);
      applyControl(u0);
    } else if (openLoopEnabled) {
      e0 = 0.0f;
      u0 = (float)openLoopPWM;
      applyControl(u0);
    } else {
      e0 = 0.0f;
      u0 = 0.0f;
      stopControl();
    }

    sendDataLine(t_s, T1_filt, T2_filt, T3_filt, T3est0, e0, u0);
  }
}

// =====================================================
// ACQUISITION + FILTRE
// =====================================================

void acquireAndFilterTemperatures() {
  uint16_t adc1 = analogRead(T1_VOLTAGE);
  uint16_t adc2 = analogRead(T2_VOLTAGE);
  uint16_t adc3 = analogRead(T3_VOLTAGE);

  float voltage1 = (adc1 * VREF) / ADC_MAX;
  float voltage2 = (adc2 * VREF) / ADC_MAX;
  float voltage3 = (adc3 * VREF) / ADC_MAX;

  float R1 = Rw1 * (5.0f * G1 - 2.0f * (voltage1 - Vref)) / (5.0f * G1 + 2.0f * (voltage1 - Vref));
  float R2 = Rw2 * (5.0f * G2 - 2.0f * (voltage2 - Vref)) / (5.0f * G2 + 2.0f * (voltage2 - Vref));
  float R3 = Rw3 * (5.0f * G3 - 2.0f * (voltage3 - Vref)) / (5.0f * G3 + 2.0f * (voltage3 - Vref));

  float T1 = computeTemperature(R1, R01, a_th, b_th, c_th, d_th);
  float T2 = computeTemperature(R2, R02, a_th, b_th, c_th, d_th);
  float T3 = computeTemperature(R3, R03, a_th, b_th, c_th, d_th);

  if (filterCount < FILTER_WINDOW) {
    t1Sum += T1;
    t2Sum += T2;
    t3Sum += T3;

    t1Buffer[filterIndex] = T1;
    t2Buffer[filterIndex] = T2;
    t3Buffer[filterIndex] = T3;

    filterCount++;
  } else {
    t1Sum -= t1Buffer[filterIndex];
    t2Sum -= t2Buffer[filterIndex];
    t3Sum -= t3Buffer[filterIndex];

    t1Buffer[filterIndex] = T1;
    t2Buffer[filterIndex] = T2;
    t3Buffer[filterIndex] = T3;

    t1Sum += T1;
    t2Sum += T2;
    t3Sum += T3;
  }

  filterIndex++;
  if (filterIndex >= FILTER_WINDOW) {
    filterIndex = 0;
  }

  T1_filt = t1Sum / filterCount;
  T2_filt = t2Sum / filterCount;
  T3_filt = t3Sum / filterCount;
}

// =====================================================
// MODELE T3 ESTIMEE
// =====================================================

void updateT3Estimate() {
  // Estimation de T3 à l'aide de T2 ainsi que de la température ambiante.
  x0 = 0.59517f * (T2_filt - T3est1)
     - 0.0975f * (T3est1 - Tamb);

  T3est0 = t3m_d1 * x1
         + t3m_d2 * x2
         - t3m_c1 * T3est1
         - t3m_c2 * T3est2;

  if (!isfinite(T3est0)) {
    T3est0 = Tamb;
  }

  x2 = x1;
  x1 = x0;

  T3est2 = T3est1;
  T3est1 = T3est0;
}-

// =====================================================
// SELECTION PIDF FIGEE PAR TRANSITION
// =====================================================

uint8_t selectPidfIndexForTransition(float startT3, float newSetpoint) {
  // Choix du PIDF à utiliser
  float delta = newSetpoint - startT3;

  if (delta < 0.0f) {
    return 1;   // PIDF 2 : tous les refroidissements
  }

  if (newSetpoint <= 23.0f) {
    return 2;   // PIDF 3 : montées basses
  }

  if (startT3 >= 30.0f || newSetpoint >= 32.0f) {
    return 0;   // PIDF 1 : montées hautes
  }

  return 3;     // PIDF 4 : montées intermédiaires
}

float computePIDDiscreteAntiWindup(float e) {
  const PIDFBankEntry &pid = pidfBank[activePidfIndex];
  float e0_try = e;
  float e1_try = e1;
  float e2_try = e2;
  float u1_try = u1;
  float u2_try = u2;

  float u_unsat =
      pid.b0 * e0_try
    + pid.b1 * e1_try
    + pid.b2 * e2_try
    - pid.a1 * u1_try
    - pid.a2 * u2_try
    + uop;
  float u_sat = constrain(u_unsat, UMIN, UMAX);

  bool blockUpdate =
      (u_unsat > UMAX && e > 0.0f) ||
      (u_unsat < UMIN && e < 0.0f);

  if (!blockUpdate) {
    e2 = e1;
    e1 = e0_try;

    u2 = u1;
    u1 = u_sat - uop;
  }
  return u_sat;
}

// =====================================================
// FONCTIONS GENERALES
// =====================================================

float computeTemperature(float R, float R0, float a, float b, float c, float d) {
  float x = log(R / R0);
  return pow(a + b * x + c * x * x + d * x * x * x, -1.0f) - 273.15f;
}

float computeUop(float Tamb, float T3_est, float y4, float y3, float y2, float y1) {
  float deltaT = T3_est - Tamb;
  return y4 * deltaT * deltaT * deltaT * deltaT + y3 * deltaT * deltaT * deltaT + y2 * deltaT * deltaT + y1 * deltaT;
}

void applyControl(float u) {
  int pwm = (int)fabs(u);
  if (pwm > 255) {
    pwm = 255;
  }

  if (u >= 0.0f) {
    analogWrite(PWM1, pwm);
    analogWrite(PWM2, 0);
  } else {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, pwm);
  }
}

void stopControl() {
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  resetControllerState();
}

void resetControllerState() {
  u0 = 0.0f;
  u1 = 0.0f;
  u2 = 0.0f;

  e0 = 0.0f;
  e1 = 0.0f;
  e2 = 0.0f;
}

void sendDataLine(float t_s, float T1, float T2, float T3, float T3_est, float e, float u) {
  Serial.print("DATA,");
  Serial.print(t_s, 2);
  Serial.print(",");
  Serial.print(T1, 2);
  Serial.print(",");
  Serial.print(T2, 2);
  Serial.print(",");
  Serial.print(T3, 2);
  Serial.print(",");
  Serial.print(T3_est, 2);
  Serial.print(",");
  Serial.print(e, 2);
  Serial.print(",");
  Serial.println(u, 2);
}

// =====================================================
// GESTION SERIE NON BLOQUANTE
// =====================================================

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      rxBuffer[rxIndex] = '\0';

      if (rxIndex > 0) {
        processCommand(rxBuffer);
      }

      rxIndex = 0;
    } else {
      if (rxIndex < RX_BUFFER_SIZE - 1) {
        rxBuffer[rxIndex++] = c;
      } else {
        rxIndex = 0;
        Serial.println("ERR,RX_OVERFLOW");
      }
    }
  }
}

void processCommand(char *line) {
  char *cmd = strtok(line, ",");

  if (cmd == nullptr) {
    Serial.println("ERR,EMPTY");
    return;
  }

  if (strcmp(cmd, "PING") == 0) {
    Serial.println("ACK,PING");
    return;
  }


  if (strcmp(cmd, "SETPOINT") == 0) {
    char *arg = strtok(nullptr, ",");
    if (arg == nullptr) {
      Serial.println("ERR,SETPOINT_FORMAT");
      return;
    }
    uop = u0;
    float newSetpoint = atof(arg);
    transitionStartT3 = T3_filt;
    activePidfIndex = selectPidfIndexForTransition(transitionStartT3, newSetpoint);
    consigne = newSetpoint;
    setpointEnabled = true;
    openLoopEnabled = false;
    openLoopPWM = 0;

    resetControllerState();

    Serial.print("ACK,SETPOINT,");
    Serial.print(consigne, 3);
    Serial.print(",PIDF_INDEX,");
    Serial.println(activePidfIndex + 1);
    return;
  }

  if (strcmp(cmd, "PWM") == 0) {
    char *arg = strtok(nullptr, ",");
    if (arg == nullptr) {
      Serial.println("ERR,PWM_FORMAT");
      return;
    }

    openLoopPWM = constrain(atoi(arg), -255, 255);
    openLoopEnabled = true;
    setpointEnabled = false;
    consigne = T3_filt;

    resetControllerState();

    Serial.print("ACK,PWM,");
    Serial.println(openLoopPWM);
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    setpointEnabled = false;
    openLoopEnabled = false;
    openLoopPWM = 0;
    stopControl();
    Serial.println("ACK,STOP");
    return;
  }

  if (strcmp(cmd, "AMBIENT") == 0) {
    char *arg = strtok(nullptr, ",");
    if (arg == nullptr) {
      Serial.println("ERR,AMBIENT_FORMAT");
      return;
    }

    Tamb = atof(arg);
    Serial.print("ACK,AMBIENT,");
    Serial.println(Tamb, 3);
    return;
  }

  if (strcmp(cmd, "PIDF_BANK") == 0) {
    for (uint8_t i = 0; i < PIDF_BANK_SIZE; ++i) {
      char *argSlot = strtok(nullptr, ",");
      char *argKp = strtok(nullptr, ",");
      char *argKi = strtok(nullptr, ",");
      char *argKd = strtok(nullptr, ",");
      char *argTf = strtok(nullptr, ",");
      char *argb0 = strtok(nullptr, ",");
      char *argb1 = strtok(nullptr, ",");
      char *argb2 = strtok(nullptr, ",");
      char *arga1 = strtok(nullptr, ",");
      char *arga2 = strtok(nullptr, ",");

      if (!argSlot || !argKp || !argKi || !argKd || !argTf ||
          !argb0 || !argb1 || !argb2 || !arga1 || !arga2) {
        Serial.println("ERR,PIDF_BANK_FORMAT");
        return;
      }

      pidfBank[i].slotId = (uint8_t)atoi(argSlot);
      pidfBank[i].kp = atof(argKp);
      pidfBank[i].ki = atof(argKi);
      pidfBank[i].kd = atof(argKd);
      pidfBank[i].tf = atof(argTf);
      pidfBank[i].b0 = atof(argb0);
      pidfBank[i].b1 = atof(argb1);
      pidfBank[i].b2 = atof(argb2);
      pidfBank[i].a1 = atof(arga1);
      pidfBank[i].a2 = atof(arga2);
    }

    resetControllerState();

    Serial.print("ACK,PIDF_BANK");
    Serial.println();
    return;
  }

  if (strcmp(cmd, "PIDF") == 0) {
    char *argKp = strtok(nullptr, ",");
    char *argKi = strtok(nullptr, ",");
    char *argKd = strtok(nullptr, ",");
    char *argTf = strtok(nullptr, ",");
    char *argb0 = strtok(nullptr, ",");
    char *argb1 = strtok(nullptr, ",");
    char *argb2 = strtok(nullptr, ",");
    char *arga1 = strtok(nullptr, ",");
    char *arga2 = strtok(nullptr, ",");

    if (!argKp || !argKi || !argKd || !argTf ||
        !argb0 || !argb1 || !argb2 || !arga1 || !arga2) {
      Serial.println("ERR,PIDF_FORMAT");
      return;
    }

    pidfBank[0].kp = atof(argKp);
    pidfBank[0].ki = atof(argKi);
    pidfBank[0].kd = atof(argKd);
    pidfBank[0].tf = atof(argTf);
    pidfBank[0].b0 = atof(argb0);
    pidfBank[0].b1 = atof(argb1);
    pidfBank[0].b2 = atof(argb2);
    pidfBank[0].a1 = atof(arga1);
    pidfBank[0].a2 = atof(arga2);

    resetControllerState();
    activePidfIndex = 0;

    Serial.print("ACK,PIDF");
    Serial.print(pidfBank[0].kp, 6); Serial.print(",");
    Serial.print(pidfBank[0].ki, 6); Serial.print(",");
    Serial.print(pidfBank[0].kd, 6); Serial.print(",");
    Serial.print(pidfBank[0].tf, 6); Serial.print(",");
    Serial.print(pidfBank[0].b0, 6); Serial.print(",");
    Serial.print(pidfBank[0].b1, 6); Serial.print(",");
    Serial.print(pidfBank[0].b2, 6); Serial.print(",");
    Serial.print(pidfBank[0].a1, 6); Serial.print(",");
    Serial.println(pidfBank[0].a2, 6);
    return;
  }

  if (strcmp(cmd, "PID") == 0) {
    char *s_b0 = strtok(nullptr, ",");
    char *s_b1 = strtok(nullptr, ",");
    char *s_b2 = strtok(nullptr, ",");
    char *s_a1 = strtok(nullptr, ",");
    char *s_a2 = strtok(nullptr, ",");

    if (!s_b0 || !s_b1 || !s_b2 || !s_a1 || !s_a2) {
      Serial.println("ERR,PID_FORMAT");
      return;
    }

    pidfBank[0].b0 = atof(s_b0);
    pidfBank[0].b1 = atof(s_b1);
    pidfBank[0].b2 = atof(s_b2);
    pidfBank[0].a1 = atof(s_a1);
    pidfBank[0].a2 = atof(s_a2);

    resetControllerState();
    activePidfIndex = 0;

    Serial.print("ACK,PID,");
    Serial.print(pidfBank[0].b0, 6); Serial.print(",");
    Serial.print(pidfBank[0].b1, 6); Serial.print(",");
    Serial.print(pidfBank[0].b2, 6); Serial.print(",");
    Serial.print(pidfBank[0].a1, 6); Serial.print(",");
    Serial.println(pidfBank[0].a2, 6);
    return;
  }

  Serial.println("ERR,UNKNOWN_CMD");
}
