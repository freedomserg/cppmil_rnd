# homework_11 — Автопілот дрона по UART + GPIO

Бортовий автопілот, що по замкненому контуру спілкується із зовнішнім **чекером**:
читає по UART телеметрію/цілі/боєприпас/конфіг, щотакт шле команди керування
`CONTROL{accel, turnRate}` ∈ [-1..1], і в момент скиду дає імпульс на лінію
**DROP (GPIO)**. Фізику інтегрує чекер — ми лише кермуємо й вчасно скидаємо.
Той самий код працює у симуляції (socat + gpio-sim) і на реальній Raspberry Pi 5.

## Архітектура

```
UART ──▶ UartLink ──▶ (Telemetry/Target/Ammo/Config)
                          │
      UartTargetStore ◀───┤ (finite-diff швидкість цілі)
                          ▼
                    MissionPlanner ── solver + стейт-машина ─▶ DroneCommand{state, angleSpeed}
                          ▼
                    DroneController ──▶ Control{accel, turnRate} ──▶ UartLink ──▶ UART
                          ▼
                    IGpio: setStart() на старті, pulseDrop() у момент скиду
```

- **Ядро наведення** (`Types`, `MathUtils`, `solvers/*`, `states/*`, `MissionPlanner`)
  перенесене з homework_10; фізику/потоки замінено на UART+GPIO I/O.
- **GPIO — два бекенди** за `IGpio`: дефолтний **сирий kernel uAPI v2** (без
  залежностей), або **libgpiod v1** через `-DUSE_GPIOD=ON` (для Pi OS Bookworm).
- **Solver** обирається аргументом `--solver` (`analytical` дефолт / `table`).

## Аргументи

| Аргумент | Дефолт | Призначення |
|---|---|---|
| `--uart <dev>` | `/tmp/ttyA` | послідовний порт (симуляція) або `/dev/ttyAMA1` на платі |
| `--gpiochip <name>` | `gpiochip1` | чип START/DROP — **фактична назва**, яку друкує чекер |
| `--start-line <n>` | `24` | лінія START |
| `--drop-line <n>` | `23` | лінія DROP |
| `--solver <s>` | `analytical` | `analytical` \| `table` |
| `--table <path>` | `data/ballistic_table.txt` | таблиця для `--solver table` |

## Збірка

```bash
# дефолт (сирий uAPI, без залежностей) — і в девконтейнері, і на Pi
cmake --preset debug && cmake --build --preset debug --target drone_autopilot

# на Pi 5 з libgpiod v1
cmake --preset debug -DUSE_GPIOD=ON && cmake --build --preset debug --target drone_autopilot
```

## Запуск — симуляція (без дротів)

Потрібен Linux/Pi 5 з root і ядром ≥5.17 (gpio-sim). Готовий скрипт:

```bash
sudo ./scripts/run_sim.sh 1 ./checker_pi_arm64        # місія 1
# змінні: GPIOCHIP=... START_LINE=24 DROP_LINE=23 SOLVER=analytical
```

Або вручну у трьох терміналах:

```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyA pty,raw,echo=0,link=/tmp/ttyB   # T1
sudo ./checker_pi_arm64 1 --uart /tmp/ttyB --start-line 24 --drop-line 23  # T2 (друкує чип)
./drone_autopilot --uart /tmp/ttyA --gpiochip <printed> --start-line 24 --drop-line 23  # T3
```

Очікуваний результат — чекер друкує `HIT`, промах ≤ `hitRadius`.

## Запуск — реальна Raspberry Pi 5 (`--hw`)

Другий UART: додати `dtoverlay=uart3` у `/boot/firmware/config.txt`, перезавантажити
→ зʼявиться `/dev/ttyAMA1`. Перемички:

| Сигнал | Напрям | Підключення |
|---|---|---|
| UART телеметрія | чекер → студ | чекер TX pin8 (GPIO14) → студ RX pin29 (GPIO5) |
| UART керування | студ → чекер | студ TX pin7 (GPIO4) → чекер RX pin10 (GPIO15) |
| START | студ → чекер | студ GPIO24 (pin18) → чекер GPIO27 (pin13) |
| DROP | студ → чекер | студ GPIO23 (pin16) → чекер GPIO22 (pin15) |

Спільна земля, 3.3 В. На Pi 5 гребінка — `gpiochip0` (на ранніх образах `gpiochip4`),
перевір `gpiodetect`.

```bash
cmake --preset debug -DUSE_GPIOD=ON && cmake --build --preset debug --target drone_autopilot
sudo ./checker 1 --hw --uart /dev/serial0 --gpiochip gpiochip0 --start-line 27 --drop-line 22
./drone_autopilot --uart /dev/ttyAMA1 --gpiochip gpiochip0 --start-line 24 --drop-line 23
```
