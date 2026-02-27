# 🚗 Veículo Microcontrolado com ESP32

Projeto de veículo robótico controlado via Bluetooth desenvolvido durante o curso técnico em Mecatrônica. O sistema foi projetado com foco em controle proporcional, estabilidade e integração entre hardware e software embarcado.

O projeto conquistou **1º lugar em uma competição interna entre equipes**, destacando-se pelo desempenho e precisão de controle.

---

## 📌 Visão geral

O veículo utiliza um **ESP32** para receber comandos de um controle **DualShock 4** via Bluetooth, permitindo controle proporcional de velocidade e direção através dos gatilhos e analógicos.

O objetivo foi construir uma plataforma robusta, responsiva e com boa capacidade de tração, aplicando conceitos de eletrônica, programação embarcada e controle de motores.

---

## 🧠 Funcionalidades

* Controle via Bluetooth com controle PS4
* Velocidade proporcional aos gatilhos
* Direção proporcional ao joystick
* Tração 4x4
* Iluminação frontal e traseira em LED
* Controle PWM independente para cada lado do veículo

---

## ⚙️ Hardware utilizado

* ESP32
* 4 motores DC 6V
* 2 drivers de motor TB6612FNG
* Pack de baterias 2S2P (Li-ion reaproveitadas)
* LEDs 5mm para iluminação

---

## 🧩 Arquitetura do sistema

O ESP32 recebe os dados do controle via Bluetooth utilizando a biblioteca **Bluepad32**.
Os valores analógicos são convertidos em sinais PWM, controlando velocidade e direção dos motores através dos drivers.

Essa abordagem permite:

* condução mais suave
* melhor controle em curvas
* resposta proporcional ao usuário

---

## 🏆 Resultados

O veículo apresentou:

* boa estabilidade
* torque elevado
* resposta rápida aos comandos

O projeto obteve **1º lugar na competição interna**, validando as decisões de projeto e implementação.

---

## 📂 Estrutura do repositório

```
veiculo-microcontrolado/
├── src/
│   └── veiculo_microcontrolado.ino   # Código principal do ESP32
└── README.md
```

---

## 🚀 Possíveis melhorias futuras

* Telemetria via Wi-Fi
* Controle por aplicativo próprio
* Sensores de distância para assistência de direção
* Suspensão otimizada
