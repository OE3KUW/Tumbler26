# Tumbler26

PlatformIO-Projekt fuer die Tumbler26-Hardware mit einem ESP32-WROOM-32E.

Ein Hardware-Timer erzeugt jede Sekunde einen Interrupt. Die Interrupt-Routine
zaehlt nur die vergangenen Sekunden; die zeitkritische WS2812B-Ausgabe erfolgt
sicher ausserhalb des Interrupts in `loop()`. Acht RGB-LEDs werden abwechselnd
in acht Farben eingeschaltet und eine Sekunde spaeter ausgeschaltet.

## Hardware

- Controller: ESP32-WROOM-32E
- RGB-LEDs: 8 x WS2812B, seriell verkettet
- Datenleitung: GPIO 15 (`RGB` im Schaltplan)
- Versorgung der LEDs: 5 V
- Datenformat: GRB, 800 kHz

> **Hardware-Hinweis:** Im Schaltplan liegt `PullUpRGB` (10 kOhm) zwischen der
> 5-V-Versorgung und der RGB-Datenleitung an GPIO 15. ESP32-GPIOs sind nicht
> 5-V-tolerant. Vor laengerem Betrieb sollte geprueft werden, ob dieser Pull-up
> tatsaechlich bestueckt ist. Falls ja, sollte er entfernt oder auf 3,3 V gelegt
> werden. Fuer eine robuste 5-V-WS2812B-Ansteuerung empfiehlt sich ein geeigneter
> 3,3-V-auf-5-V-Pegelwandler.

## Bauen und Hochladen

```bash
pio run
pio run --target upload
pio device monitor
```

Die Helligkeit, LED-Anzahl und Datenleitung koennen in
`include/led_config.h` angepasst werden.

