// ===================== INCLUDES ===================== //
#include <Bluepad32.h>
#include <LiquidCrystal.h>


// ===================== DEFINES ===================== //
// CONFIG
#define JOYSTICK_MIN -508   // Valor mínimo do joystick
#define JOYSTICK_MAX 508    // Valor máximo do joystick
#define DEADZONE 50         // DEADZONE joystick
#define DIFERENCIAL_MAX 128  // Diferença máxima entre PWM
#define PWM_MIN 60           // PWM mínimo de motores

// PINS
#define FRONTAL_ESQ_IN1 26
#define FRONTAL_ESQ_IN2 25
#define FRONTAL_ESQ_PWM 17

#define FRONTAL_DIR_IN1 16
#define FRONTAL_DIR_IN2 27
#define FRONTAL_DIR_PWM 14

#define FRONTAL_STANDBY 12

#define TRASEIRO_ESQ_IN1 13
#define TRASEIRO_ESQ_IN2 5
#define TRASEIRO_ESQ_PWM 23

#define TRASEIRO_DIR_IN1 19
#define TRASEIRO_DIR_IN2 18
#define TRASEIRO_DIR_PWM 2

#define TRASEIRO_STANDBY 4


// ===================== CLASSES ===================== //
class Led {
private:
  int pin;
  bool state;

public:
  Led(int pin) {
    this->pin = pin;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    state = false;
  }

  void on() {
    if (!state) {
      Serial.printf("LED %d ON \n", pin);
      digitalWrite(pin, HIGH);
      state = true;
    }
  }

  void off() {
    if (state) {
      Serial.printf("LED %d OFF \n", pin);
      digitalWrite(pin, LOW);
      state = false;
    }
  }

  void pwm(unsigned int value) {
    Serial.print("PWM: ");
    Serial.println(value);
    analogWrite(pin, constrain(value, 0, 255));
  }
};


// ===================== VARIÁVEIS GLOBAIS ===================== //
// Controle bluetooth
ControllerPtr myControllers[BP32_MAX_GAMEPADS];


const int pwmChannel_esq = 0;
const int pwmChannel_dir = 1;
const int pwmFreq = 5000;     // Frequência em Hz
const int pwmResolution = 8;  // 8 bits = valores de 0 a 255


int brake_pwm = 0;
int throttle_pwm = 0;
int analog_esq = 0;
int vel_base = 0;
int vel_esq = 0;
int vel_dir = 0;
bool sentido = 1;


// ===================== PROTÓTIPOS DE FUNÇÕES ===================== //
void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void dumpGamepad(ControllerPtr ctl);
void processGamepad(ControllerPtr ctl);
void processControllers();


// ===================== FUNÇÃO SETUP ===================== //
void setup() {
  // Inicializa a comunicação serial e configura o Bluepad32
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Configura os callbacks do Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" deve ser chamado quando o usuário realizar um "reset de fábrica"
  BP32.forgetBluetoothKeys();

  // Desabilita dispositivos virtuais (exemplo: mouse ou touchpad em alguns controles)
  BP32.enableVirtualDevice(false);

  // configura o canal
  ledcSetup(pwmChannel_esq, pwmFreq, pwmResolution);
  ledcSetup(pwmChannel_dir, pwmFreq, pwmResolution);

  // associa o canal ao pino
  ledcAttachPin(FRONTAL_ESQ_PWM, pwmChannel_esq);
  ledcAttachPin(FRONTAL_DIR_PWM, pwmChannel_dir);
  ledcAttachPin(TRASEIRO_ESQ_PWM, pwmChannel_esq);
  ledcAttachPin(TRASEIRO_DIR_PWM, pwmChannel_dir);


  pinMode(FRONTAL_ESQ_IN1, OUTPUT);
  pinMode(FRONTAL_ESQ_IN2, OUTPUT);
  pinMode(FRONTAL_DIR_IN1, OUTPUT);
  pinMode(FRONTAL_DIR_IN2, OUTPUT);
  pinMode(FRONTAL_STANDBY, OUTPUT);
  pinMode(TRASEIRO_ESQ_IN1, OUTPUT);
  pinMode(TRASEIRO_ESQ_IN2, OUTPUT);
  pinMode(TRASEIRO_DIR_IN1, OUTPUT);
  pinMode(TRASEIRO_DIR_IN2, OUTPUT);
  pinMode(TRASEIRO_STANDBY, OUTPUT);
}


// ===================== FUNÇÃO LOOP ===================== //
void loop() {
  // Atualiza os dados dos controles conectados
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }

  Serial.print("Vel base: ");
  Serial.print(vel_base);
  Serial.print(" | Vel esquerda: ");
  Serial.print(vel_esq);
  Serial.print(" | Vel direita: ");
  Serial.print(vel_dir);
  Serial.print(" | Sentido: ");
  Serial.print(sentido);
  Serial.print(" | Brake pwm ");
  Serial.print(brake_pwm);
  Serial.print(" | Throttle pwm: ");
  Serial.print(throttle_pwm);
  Serial.print(" | Analógico esquerdo: ");
  Serial.println(analog_esq);

  delay(10);

  controlMotores();

  /*
  Preciso de um código que no momento que eu usar o joystick, ele ativa uma função, 
  que será executada enquanto o joystick estiver no movimento certo.
  No momento em que eu soltar o joystick, a função deixe de ser executada.
  */
}


// ===================== CALLBACKS ===================== //
void onConnectedController(ControllerPtr ctl) {
  // Chamada quando um novo controle é conectado
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                    ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not find empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  // Chamada quando um controle é desconectado
  bool foundController = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }
  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}


// ===================== FUNÇÕES AUXILIARES ===================== //
void controlMotores() {
  // Standby para vel_base maior que 0

  if (vel_base > PWM_MIN) {
    ligar_chip();
    ledcWrite(pwmChannel_esq, vel_esq);
    ledcWrite(pwmChannel_dir, vel_dir);
    if (sentido == 1) {
      marchaFrente();
    }
    if (sentido == 0) {
      marchaRe();
    }
  }
  if (vel_base == 0) {
    // Deixar livre o motor, sem usar um freio
    parar();
    ledcWrite(pwmChannel_esq, 255);
    ledcWrite(pwmChannel_dir, 255);
    desligar_chip();
  }
}

void ligar_chip() {
  digitalWrite(FRONTAL_STANDBY, HIGH);
  digitalWrite(TRASEIRO_STANDBY, HIGH);
}
void desligar_chip() {
  digitalWrite(FRONTAL_STANDBY, LOW);
  digitalWrite(TRASEIRO_STANDBY, LOW);
}

void marchaFrente() {
  digitalWrite(FRONTAL_ESQ_IN1, LOW);
  digitalWrite(FRONTAL_ESQ_IN2, HIGH);

  digitalWrite(FRONTAL_DIR_IN1, HIGH);
  digitalWrite(FRONTAL_DIR_IN2, LOW);

  digitalWrite(TRASEIRO_ESQ_IN1, LOW);
  digitalWrite(TRASEIRO_ESQ_IN2, HIGH);
  digitalWrite(TRASEIRO_DIR_IN1, LOW);
  digitalWrite(TRASEIRO_DIR_IN2, HIGH);
}
void marchaRe() {
  digitalWrite(FRONTAL_ESQ_IN1, HIGH);
  digitalWrite(FRONTAL_ESQ_IN2, LOW);

  digitalWrite(FRONTAL_DIR_IN1, LOW);
  digitalWrite(FRONTAL_DIR_IN2, HIGH);

  digitalWrite(TRASEIRO_ESQ_IN1, HIGH);
  digitalWrite(TRASEIRO_ESQ_IN2, LOW);
  digitalWrite(TRASEIRO_DIR_IN1, HIGH);
  digitalWrite(TRASEIRO_DIR_IN2, LOW);
}

void freio() {
}
void parar() {
  digitalWrite(FRONTAL_ESQ_IN1, LOW);
  digitalWrite(FRONTAL_ESQ_IN2, LOW);
  digitalWrite(FRONTAL_DIR_IN1, LOW);
  digitalWrite(FRONTAL_DIR_IN2, LOW);

  digitalWrite(TRASEIRO_ESQ_IN1, LOW);
  digitalWrite(TRASEIRO_ESQ_IN2, LOW);
  digitalWrite(TRASEIRO_DIR_IN1, LOW);
  digitalWrite(TRASEIRO_DIR_IN2, LOW);
}


void processGamepad(ControllerPtr ctl) {
  // == Gatilho Esquerdo ==
  brake_pwm = map(ctl->brake(), 0, 1020, 0, 255);

  // == Gatilho Direito ==
  throttle_pwm = map(ctl->throttle(), 0, 1020, 0, 255);

  if (throttle_pwm > 0 || brake_pwm > 0) {
    if (throttle_pwm >= brake_pwm) {
      sentido = 1;
      vel_base = throttle_pwm;
    } else {
      sentido = 0;
      vel_base = brake_pwm;
    }
  } else {
    vel_base = 0;  // Nenhum gatilho pressionado
    // sentido permanece com o último valor
  }

  analog_esq = ctl->axisX();

  //== LEFT JOYSTICK DEADZONE ==//
  if (ctl->axisX() >= -DEADZONE && ctl->axisX() <= DEADZONE) {
    vel_esq = vel_dir = vel_base;
  }

  // == LEFT JOYSTICK OUT DEADZONE ==//
  if (ctl->axisX() < -DEADZONE || ctl->axisX() > DEADZONE) {
    int diferenca = map(abs(ctl->axisX()), DEADZONE, 508, 0, DIFERENCIAL_MAX);
    if (ctl->axisX() > 0) {  // ir para direita
      // Diminuir velocidade da direita
      vel_dir = constrain(vel_base - diferenca, 0, 255);
      vel_esq = vel_base;
    }
    if (ctl->axisX() < 0) {  // ir para a esquerda
      // Diminuir velocidade da esquerda
      vel_esq = constrain(vel_base - diferenca, 0, 255);
      vel_dir = vel_base;
    }
  }


  // // == PS4 X button ==//
  // if (ctl->buttons() & 0x0001) {
  // }
  // //== PS4 Circle button ==//
  // if (ctl->buttons() & 0x0002) {
  // }
  // //== PS4 Square button ==//
  // if (ctl->buttons() & 0x0004) {
  // }
  // //== PS4 Triangle button ==//
  // if (ctl->buttons() & 0x0008) {
  // }


  // //== PS4 Dpad UP button ==//
  // if (ctl->dpad() & 0x01) {
  // }
  // //== PS4 Dpad DOWN button ==//
  // if (ctl->dpad() & 0x02) {
  // }
  // //== PS4 Dpad LEFT button ==//
  // if (ctl->dpad() & 0x08) {
  // }
  // //== PS4 Dpad RIGHT button ==//
  // if (ctl->dpad() & 0x04) {
  // }


  // //== PS4 L1 button ==//
  // if (ctl->buttons() & 0x0010) {
  // }
  // //== PS4 L2 button ==//
  // if (ctl->buttons() & 0x0040) {
  // }
  // //== PS4 L3 button ==//
  // if (ctl->buttons() & 0x0100) {
  // }


  // //== PS4 R1 button ==//
  // if (ctl->buttons() & 0x0020) {
  // }
  // //== PS4 R2 button ==//
  // if (ctl->buttons() & 0x0080) {
  // }
  // //== PS4 R3 button ==//
  // if (ctl->buttons() & 0x0200) {
  // }


  // //== LEFT JOYSTICK - UP ==//
  // if (ctl->axisY() < -DEADZONE) {
  // }
  // //== LEFT JOYSTICK - DOWN ==//
  // if (ctl->axisY() > DEADZONE) {
  // }
  // //== LEFT JOYSTICK - LEFT ==//
  // if (ctl->axisX() < -DEADZONE) {
  // }
  // //== LEFT JOYSTICK - RIGHT ==//
  // if (ctl->axisX() > DEADZONE) {
  // }


  // //== RIGHT JOYSTICK - UP ==//
  // if (ctl->axisRY() < -DEADZONE) {
  // }
  // //== RIGHT JOYSTICK - DOWN ==//
  // if (ctl->axisRY() > DEADZONE) {
  // }
  // //== RIGHT JOYSTICK - RIGHT ==//
  // if (ctl->axisRX() < -DEADZONE) {
  // }
  // //== RIGHT JOYSTICK - RIGHT ==//
  // if (ctl->axisRX() > DEADZONE) {
  // }


  // // == LEFT JOYSTICK OUT DEADZONE ==//
  // if (ctl->axisY() < -DEADZONE || ctl->axisY() > DEADZONE || ctl->axisX() < -DEADZONE || ctl->axisX() > DEADZONE) {
  //   Serial.printf("Left Joystick: X %d | Y %d \n", ctl->axisX(), ctl->axisY());
  // }

  // // == RIGHT JOYSTICK OUT DEADZONE ==//
  // if (ctl->axisRY() < -DEADZONE || ctl->axisRY() > DEADZONE || ctl->axisRX() < -DEADZONE || ctl->axisRX() > DEADZONE) {
  //   Serial.printf("Right Joystick: X %d | Y %d \n", ctl->axisRX(), ctl->axisRY());
  // }



  // //== LEFT JOYSTICK DEADZONE ==//
  // if (ctl->axisY() >= -DEADZONE && ctl->axisY() <= DEADZONE && ctl->axisX() >= -DEADZONE && ctl->axisX() <= DEADZONE) {
  // }
  // //== RIGHT JOYSTICK DEADZONE ==//
  // if (ctl->axisRY() >= -DEADZONE && ctl->axisRY() <= DEADZONE && ctl->axisRX() >= -DEADZONE && ctl->axisRX() <= DEADZONE) {
  // }



  // Optional: Uncomment the line below to monitor all inputs
  // dumpGamepad(ctl);
}

void dumpGamepad(ControllerPtr ctl) {
  // Imprime os valores do controle conectado
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}

void processControllers() {
  // Itera pelos controles conectados e processa os dados
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}
