# jk_bms_ble – JK-BMS über Bluetooth LE für OpenMower (ROS 1 Noetic)

catkin-Paket, das ein JK-BMS (Jikong, ausgelegt auf **JK-BD4A8S4P**, hw 11.x / sw 11.x)
per BLE ausliest und als `sensor_msgs/BatteryState` publiziert – optional zusätzlich als
`mower_msgs/Bms` für die OpenMower-Logik.

## Architektur

```
┌──────────── Thread "jk_bms_ble" ────────────┐        ┌──────── rospy-Hauptthread ────────┐
│ eigener asyncio-Loop + bleak                 │        │ rospy.Rate(publish_rate_hz)       │
│  scan → connect → notify(FFE1)               │ queue. │  Queue leeren → Zustand           │
│  TX 0x97 (Device Info) → Protokoll wählen    │ Queue  │  /battery_state  (BatteryState)   │
│  TX 0x96 (Cell Info)  → BMS streamt Frames   │ ─────► │  /ll/bms         (optional)       │
│  FrameAssembler: Chunks → 300-Byte-Frame     │        │  /diagnostics    (optional)       │
│  Reconnect alle 5 s, Re-Request bei Stille   │        └───────────────────────────────────┘
└──────────────────────────────────────────────┘
```

| Datei | Inhalt |
|---|---|
| `src/jk_bms_ble/protocol.py` | Kommandos, Checksumme, Frame-Reassembly, Decoder (JK02_32S/24S) – ohne ROS/bleak |
| `src/jk_bms_ble/ble_worker.py` | BLE-Thread (bleak/asyncio), Reconnect, Watchdog |
| `src/jk_bms_ble/battery_logic.py` | Mapping auf `power_supply_status` / `power_supply_health` |
| `src/jk_bms_ble/node.py` | ROS-Node, Publishing, Verbindungsabbruch-Verhalten |
| `test/` | Unit-Tests mit echten JK-Frames (aus dem esphome-jk-bms-Testsuite) |

## Wichtige Erkenntnisse / bewusste Abweichungen vom Prompt

1. **`aiobmsble` ist unter Noetic nicht nutzbar.** Alle Versionen verlangen Python ≥ 3.12,
   ROS Noetic läuft auf Python 3.8 (Ubuntu 20.04). Das Parsing ist daher eigenständig
   implementiert, Offsets 1:1 an `aiobmsble/bms/jikong_bms.py` und `esphome-jk-bms`
   ausgerichtet und gegen **echte Frames** (fw 10.07 / 14.20 / 15.38) inkl. der dort
   annotierten Sollwerte getestet.
2. **Protokoll:** JK02_32S, Kommando-Header `AA 55 90 EB`, Antwort-Header `55 AA EB 90`,
   Bytesumme mod 256, Frame 300 Byte. Firmware ≥ 11 → JK02_32S (automatisch aus dem
   Device-Info-Frame, `protocol: auto`). `0x4E 0x57` (RS485) wird nicht verwendet.
3. **Welches Topic nutzt OpenMower wirklich?** Geprüft in `open_mower_ros` (upstream `main`,
   Commit `224c8a6`, Sep. 2026 – bitte gegen euren installierten Stand v1.2 gegenprüfen):
   - `mower_logic` abonniert **`/ll/power`** (`mower_msgs/Power`) und **`/ll/bms`** (`mower_msgs/Bms`).
     `sensor_msgs/BatteryState` wird von OpenMower **nicht** konsumiert.
   - Batteriespannung für Docking/Kritisch-Schwelle:
     `GetFirstValid({power.battery_voltage_adc, bms.voltage, power.battery_voltage})` –
     der **ADC-Wert des Mainboards hat Vorrang**. `/ll/bms` greift nur, wenn dieser ungültig
     (0/NaN) ist.
   - `/ll/power` wird von `mower_comms` publiziert und enthält die Ladespannung für die
     Dock-Erkennung. Ein zweiter Publisher dort würde das Docking stören → der Node
     **verweigert** `openmower_status_topic: /ll/power`.
   - Mit `openmower_status_topic: "/ll/bms"` publiziert der Node zusätzlich `mower_msgs/Bms`.
     Achtung: `mower_comms_v2` advertised `/ll/bms` ebenfalls (BMS-Service der Firmware).
     Vorher mit `rostopic info /ll/bms` prüfen, ob dort schon jemand publiziert.
   - Bei > 30 s ohne Daten werden auf `/ll/bms` NaN-Werte gesendet, damit `mower_logic`
     keine veraltete Spannung weiterverwendet (es behält sonst die letzte Nachricht ewig).
4. **Passwort (`ble_connect_password`):** Weder aiobmsble noch esphome-jk-bms senden ein
   Passwort; das JK02-BLE-Protokoll hat keine Authentifizierung auf BLE-Ebene, die PIN prüft
   nur die JK-App bei Einstellungsänderungen. Der Parameter existiert, wird aber nicht
   gesendet (Info-Log). **Beim ersten Hardwaretest verifizieren** – falls das BMS ohne PIN
   keine Daten liefert, bitte melden.
5. **Status-Reihenfolge:** `DISCHARGING` (I < −0,1 A) → `FULL` (SoC = 100 %) → `CHARGING`
   (I > 0,1 A) → `NOT_CHARGING`. `FULL` hat damit Vorrang vor kleinem Balancer-/Erhaltungsstrom.
6. `mower_msgs/Bms.relative_state_of_charge` wird in **Prozent (0–100)** befüllt
   (SMBus-Konvention); die Einheit ist in `open_mower_ros` nicht dokumentiert.

## Nachrichtenbelegung `sensor_msgs/BatteryState`

| Feld | Quelle |
|---|---|
| `voltage` / `current` | Gesamtspannung / Strom (+ Laden, − Entladen) |
| `percentage` | SoC / 100 |
| `charge` | Restkapazität vom BMS (Fallback: SoC × capacity) |
| `capacity` | Vollladekapazität vom BMS (Fallback: `nominal_capacity_ah`) |
| `design_capacity` | `nominal_capacity_ah` |
| `temperature` | MOSFET-Temperatur |
| `cell_voltage` | erste `cell_count` Zellen (fehlende = NaN) |
| `present` | BLE verbunden |
| `power_supply_health` | GOOD; DEAD (Unterspannung), OVERVOLTAGE, OVERHEAT, COLD, UNSPEC_FAILURE je nach JK-Fehlerbit; UNKNOWN nach > `stale_timeout_s` ohne Daten |
| `serial_number` | aus dem Device-Info-Frame |

NTC T1/T2 („NA“, wenn nicht angeschlossen → `None`), MOSFET-Zustände, Fehlerliste, Zyklen
und SOH stehen auf `/diagnostics` (und im `extra_data`-JSON von `/ll/bms`).

**Verbindungsabbruch:** Letzte Werte werden weiter publiziert, `present=False`, Warn-Log.
Nach > 30 s zusätzlich `power_supply_health = UNKNOWN`. Vor dem ersten Datensatz: NaN-Werte.

## Parameter

| Parameter | Default | Beschreibung |
|---|---|---|
| `bms_mac_address` | – (Pflicht) | BLE-MAC des BMS |
| `publish_rate_hz` | `1.0` | Publish-Rate |
| `cell_count` | `7` | Zellen in `cell_voltage` |
| `nominal_capacity_ah` | `10.0` | `design_capacity` |
| `ble_connect_password` | `""` | siehe oben, wird nicht gesendet |
| `openmower_status_topic` | `""` | z. B. `/ll/bms` → zusätzlich `mower_msgs/Bms` |
| `battery_state_topic` | `/battery_state` | |
| `protocol` | `auto` | `auto`, `JK02_32S`, `JK02_24S` |
| `frame_id` | `battery` | |
| `publish_diagnostics` | `true` | `/diagnostics` |
| `reconnect_interval_s` | `5.0` | Wartezeit zwischen Verbindungsversuchen |
| `stale_timeout_s` | `30.0` | danach Health UNKNOWN |
| `data_timeout_s` | `10.0` | Stille → 0x96 erneut senden, nach 3× Reconnect |

## Installation (Raspberry Pi CM4, ROS Noetic)

```bash
# 1. Abhängigkeit (Python 3.8 → bleak 0.22.x)
pip3 install -r requirements.txt

# 2. In den Workspace legen und bauen
cd ~/catkin_ws/src   # bzw. der open_mower_ros-Workspace
git clone https://github.com/kiwi220/BMS-Openmower.git jk_bms_ble
cd .. && catkin_make   # oder: catkin build jk_bms_ble
source devel/setup.bash

# 3. MAC finden (Name z. B. "JK-BD4A8S-4P"); JK-App vorher schließen –
#    das BMS akzeptiert nur eine BLE-Verbindung gleichzeitig!
bluetoothctl --timeout 15 scan on

# 4. Starten
roslaunch jk_bms_ble jk_bms.launch bms_mac_address:=C8:47:80:XX:XX:XX
rostopic echo /battery_state
```

**OpenMower im Container:** Auf den OpenMower-Images läuft ROS üblicherweise in einem
Container. bleak spricht über D-Bus mit `bluetoothd` auf dem Host – der Container braucht
dann Zugriff auf den System-Bus (z. B. `-v /run/dbus:/run/dbus:ro`), und das Paket muss im
Container-Workspace gebaut bzw. `bleak` dort installiert sein. Details hängen vom konkreten
Image ab.

## Erste Inbetriebnahme – Checkliste

- [ ] `rosrun jk_bms_ble jk_bms_node _bms_mac_address:=… _publish_rate_hz:=1.0` → Log
      „Connected to JK-BMS“ und „BMS model …, sw 11.24 … → protocol JK02_32S“
- [ ] Werte in `/battery_state` mit der JK-App vergleichen (Spannung, Strom-Vorzeichen, SoC)
- [ ] Temperaturen T1/T2 in `/diagnostics` (`NA` wenn nicht gesteckt)
- [ ] Passwort-Frage klären (siehe oben)
- [ ] BMS-Bluetooth kurz abschalten/außer Reichweite → `present=False`, nach 30 s `UNKNOWN`,
      danach automatischer Reconnect
- [ ] Nur falls gewünscht: `openmower_status_topic: /ll/bms`, vorher `rostopic info /ll/bms`

## Tests

```bash
cd test && python3 -m unittest test_protocol test_battery_logic test_ble_worker
# oder im Workspace: catkin_make run_tests_jk_bms_ble
```

Die BLE-Worker-Tests laufen gegen ein simuliertes bleak (Chunking, AT-Nachrichten,
getrennte Write/Notify-Characteristics, Verbindungsabbruch, Reconnect, Watchdog).

## Referenzen

- [patman15/BMS_BLE-HA](https://github.com/patman15/BMS_BLE-HA) / aiobmsble `jikong_bms.py` (Apache-2.0)
- [syssi/esphome-jk-bms](https://github.com/syssi/esphome-jk-bms) `jk_bms_ble` (Apache-2.0) – Testframes in `test/fixtures.py`
- [open_mower_ros](https://github.com/ClemensElflein/open_mower_ros) `mower_msgs`, `mower_logic`
