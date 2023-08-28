# GPS_node

Package with the code responsible for starting the gps node.

* **`gps_serial.ino:`** : This node is responsible for starting the GPS via ESP32 and publishing a topic with the latitude and logitude information received by the GPS.


## GPS info:

Modulo do gps: GY-NEO6MV2
- Tensão de operação: 2,7 a 5VDC
- Nível lógico: 3.3 e 5V
- Pode rastrear até 22 satélites em 50 canais
- Utilizar em local aberto (janela)

## Esp32 > GY-NEO6
- 3.3v > vcc
- gng > gnd
- rx > tx
- tx > rx
