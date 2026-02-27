# 🚗 Veículo Microcontrolado com ESP32 e Controle Bluetooth

Projeto desenvolvido durante o curso técnico em Mecatrônica, com foco em sistemas embarcados, controle de motores e comunicação sem fio.
O veículo foi construído para uma competição prática entre equipes, conquistando o **1º lugar** nas provas realizadas.

---

## 📌 Objetivo do projeto

Desenvolver um veículo controlado remotamente com:

* controle proporcional de velocidade
* comunicação Bluetooth estável
* boa relação entre torque, aceleração e autonomia
* arquitetura simples e robusta para testes e manutenção

O projeto serviu como aplicação prática de eletrônica, programação embarcada e integração hardware-software.

---

## ⚙️ Arquitetura do sistema

* **Microcontrolador:** ESP32
* **Comunicação:** Bluetooth clássico
* **Controle:** Gamepad DualShock 4
* **Biblioteca:** Bluepad32

### Hardware

* Tração 4x4 com motores DC 6V
* 2 drivers TB6612 (ponte H dupla)
* 4 LEDs brancos 5 mm (farol e lanterna)
* Pack de baterias 2S2P reaproveitado de power bank
  *(na fase inicial, utilizado em configuração 2S1P)*

---

## 🎮 Controle do veículo

O controle é feito via gamepad com leitura analógica:

* Gatilhos → controle proporcional de velocidade
* Joysticks → direção e variação de potência
* PWM ajustado dinamicamente conforme intensidade do acionamento

Isso permite aceleração progressiva e condução mais precisa em comparação a controles digitais simples.

---

## 🧠 Conceitos aplicados

* Controle de motores DC via PWM
* Comunicação Bluetooth embarcada
* Leitura analógica e mapeamento de sinais
* Integração hardware-software
* Testes práticos de desempenho e autonomia

---

## 🛠️ Como usar

1. Instale a IDE Arduino ou PlatformIO
2. Instale a biblioteca **Bluepad32**
3. Configure a placa ESP32
4. Compile e envie o código para o microcontrolador
5. Conecte o DualShock 4 via Bluetooth
6. Energize o veículo e realize os testes

---

## 📂 Estrutura do repositório

```
veiculo-microcontrolado/
 ├── src/
     └── veiculo_microcontrolado.ino
 └── README.md
```

---

## 📈 Possíveis melhorias futuras

* Controle de corrente dos motores
* Telemetria via Bluetooth/Wi-Fi
* Controle PID de velocidade
* Chassi modular impresso em 3D
* Monitoramento de bateria

---

## 👨‍💻 Autor

Henrique Sc
Estudante de Engenharia de Controle e Automação
Técnico em Mecatrônica

---

## 📜 Licença

Este projeto é aberto para fins educacionais e experimentais.
