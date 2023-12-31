# Memory Benchmark

The `MemoryBenchmark.ino` was compiled with each `FEATURE_*` and the flash
memory and static RAM sizes were recorded. The `FEATURE_BASELINE` selection is
the baseline, and its memory usage numbers are subtracted from the subsequent
`FEATURE_*` memory usage.

**Version**: AUnit v1.7.1

**DO NOT EDIT**: This file was auto-generated using `make README.md`.

## How to Regenerate

To regenerate this README.md:

```
$ make clean_benchmarks
$ make benchmarks
$ make README.md
```

The `make benchmarks` target uses `collect.sh` script which calls `auniter.sh`
(https://github.com/bxparks/AUniter) to invoke the Arduino IDE programmatically.
It produces a `*.txt` file with the flash and ram usage information (e.g.
`nano.txt`).

The `make README.md` command calls the `generated_readme.py` Python script which
generates this `README.md` file. The ASCII tables below are generated by the
`generate_table.awk` script, which takes each `*.txt` file and converts it to an
ASCII table.

## Library Size Changes

## Arduino Nano

* 16MHz ATmega328P
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* Arduino AVR Boards 1.8.6

```
+---------------------------------------------------------------------+
| Functionality                          |  flash/  ram |       delta |
|----------------------------------------+--------------+-------------|
| Baseline                               |   1586/  185 |     0/    0 |
|----------------------------------------+--------------+-------------|
| AUnit Single Test                      |   4456/  366 |  2870/  181 |
| AUnit Single Test Verbose              |   4500/  366 |  2914/  181 |
+---------------------------------------------------------------------+

```

## Sparkfun Pro Micro

* 16 MHz ATmega32U4
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* SparkFun AVR Boards 1.1.13

```
+---------------------------------------------------------------------+
| Functionality                          |  flash/  ram |       delta |
|----------------------------------------+--------------+-------------|
| Baseline                               |   3572/  150 |     0/    0 |
|----------------------------------------+--------------+-------------|
| AUnit Single Test                      |   6410/  329 |  2838/  179 |
| AUnit Single Test Verbose              |   6454/  329 |  2882/  179 |
+---------------------------------------------------------------------+

```

## SAMD21 Seeeduino XIAO M0

* SAMD51, 120 MHz ARM Cortex-M4
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* Seeeduino SAMD 1.8.4

```
+---------------------------------------------------------------------+
| Functionality                          |  flash/  ram |       delta |
|----------------------------------------+--------------+-------------|
| Baseline                               |  34164/    0 |     0/    0 |
|----------------------------------------+--------------+-------------|
| AUnit Single Test                      |  36436/    0 |  2272/    0 |
| AUnit Single Test Verbose              |  36492/    0 |  2328/    0 |
+---------------------------------------------------------------------+

```

## STM32 Blue Pill

* STM32F103C8, 72 MHz ARM Cortex-M3
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* STM32duino 2.5.0

```
+---------------------------------------------------------------------+
| Functionality                          |  flash/  ram |       delta |
|----------------------------------------+--------------+-------------|
| Baseline                               |  21540/ 3836 |     0/    0 |
|----------------------------------------+--------------+-------------|
| AUnit Single Test                      |  23648/ 3904 |  2108/   68 |
| AUnit Single Test Verbose              |  23704/ 3904 |  2164/   68 |
+---------------------------------------------------------------------+

```

## SAMD51 Adafruit ItsyBitsy M4

* SAMD51, 120 MHz ARM Cortex-M4
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* Adafruit SAMD 1.7.11

```
+---------------------------------------------------------------------+
| Functionality                          |  flash/  ram |       delta |
|----------------------------------------+--------------+-------------|
| Baseline                               |  10664/    0 |     0/    0 |
|----------------------------------------+--------------+-------------|
| AUnit Single Test                      |  12816/    0 |  2152/    0 |
| AUnit Single Test Verbose              |  12872/    0 |  2208/    0 |
+---------------------------------------------------------------------+

```

## ESP8266

* NodeMCU 1.0, 80MHz ESP8266
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* ESP8266 Boards 3.0.2

```
+---------------------------------------------------------------------+
| Functionality                          |  flash/  ram |       delta |
|----------------------------------------+--------------+-------------|
| Baseline                               | 264949/27984 |     0/    0 |
|----------------------------------------+--------------+-------------|
| AUnit Single Test                      | 268021/28148 |  3072/  164 |
| AUnit Single Test Verbose              | 268081/28148 |  3132/  164 |
+---------------------------------------------------------------------+

```

## ESP32

* ESP32-01 Dev Board, 240 MHz Tensilica LX6
* Arduino IDE 1.8.19, Arduino CLI 0.33.0
* ESP32 Boards 2.0.9

```
+---------------------------------------------------------------------+
| Functionality                          |  flash/  ram |       delta |
|----------------------------------------+--------------+-------------|
| Baseline                               | 259189/22352 |     0/    0 |
|----------------------------------------+--------------+-------------|
| AUnit Single Test                      | 265273/22448 |  6084/   96 |
| AUnit Single Test Verbose              | 265341/22448 |  6152/   96 |
+---------------------------------------------------------------------+

```

RAM usage remains constant as more objects are created, which indicates that an
initial pool of a certain minimum size is created regardless of the actual RAM
usage by objects.

