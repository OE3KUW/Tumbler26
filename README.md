# Tumbler26

PlatformIO-Firmware fuer die Tumbler26-Hardware mit ESP32-WROOM-32E. Diese
Version implementiert ausschliesslich den Modus **Balancieren**.

## Erkennungsmerkmal

LED 7 der acht WS2812B leuchtet dauerhaft **Magenta**. Alle anderen
Multicolor-LEDs bleiben aus. Dadurch ist die Balance-Firmware sofort von der
vorherigen LED-Demonstration unterscheidbar.

## Regelung

- MPU6050/GY-521 an GPIO 21 (SDA) und GPIO 22 (SCL)
- Hardwaretimer mit 5 ms Periodendauer (200 Hz)
- I2C und Regelberechnung laufen in einem hoch priorisierten FreeRTOS-Task,
  nicht in der Timer-ISR
- Winkel-PD-Regler mit 200 Hz
- Geschwindigkeits-PI-Regler mit 25 Hz
- Gierdämpfung ueber die Z-Gyroskoprate
- Quadraturauswertung der Motorencoder
- automatische Ermittlung der Encoderpolaritaet; bis dahin wird wie in der
  bewaehrten AVR-Firmware die befohlene Motordrehrichtung verwendet

Die Startwerte der Regelparameter stammen aus `OE3KUW/TumbllerVSC` und stehen
in `include/balance_config.h`.

## Pinbelegung

| Funktion | GPIO |
|---|---:|
| RGB-Datenleitung | 15 |
| PWMA / M2 links | 4 |
| AIN1 / M2 links | 16 |
| STBY | 17 |
| PWMB / M1 rechts | 18 |
| BIN1 / M1 rechts | 23 |
| M2A / M2B | 19 / 25 |
| M1A / M1B | 27 / 26 |
| SDA / SCL | 21 / 22 |

`AIN2` und `BIN2` werden auf der Hardware durch den Doppel-Inverter U23
erzeugt. Treiberkanal A ist mit M2 verbunden, Treiberkanal B mit M1.

## Startablauf

1. Nach dem Einschalten bleiben die Motoren deaktiviert.
2. Den Roboter waehrend der etwa zwei Sekunden dauernden Gyro-Kalibrierung
   ruhig halten.
3. Anschliessend den Roboter aufrecht halten.
4. Wenn der Winkel eine Sekunde lang innerhalb von +/-7 Grad liegt, wird der
   Motortreiber aktiviert.
5. Bei mehr als +/-22 Grad, mehreren MPU-Fehlern oder stark verpassten
   Regelperioden werden die Motoren abgeschaltet.
6. Nach einem Sturz kann erneut aktiviert werden, indem der Roboter wieder eine
   Sekunde aufrecht gehalten wird.

Der serielle Monitor mit 115200 Baud zeigt Winkel, Reglerausgaenge,
Encoderstaende, gelernte Encoderpolaritaet und verpasste Regelperioden.

## Bauen und Hochladen

```bash
/home/kuran/.platformio/penv/bin/pio run
/home/kuran/.platformio/penv/bin/pio run --target upload
/home/kuran/.platformio/penv/bin/pio device monitor
```

## Bewusst akzeptierter Hardwarestand

Diese Firmware ist auf ausdruecklichen Wunsch fuer den unveraenderten
Hardwarestand erstellt. Im aktuellen Schaltplan bestehen unter anderem noch
ungeklaerte beziehungsweise nicht normgerechte 5-V-Pegel an I2C, Encodern,
RGB-Datenleitung und Motortreiber-Logik. Die Software kann daraus entstehende
elektrische Schaeden oder unzuverlaessige Logikpegel nicht verhindern.

