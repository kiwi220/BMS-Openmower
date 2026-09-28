# bms_ble – BLE-Batteriemanagement (JK + ANT) für OpenMower (ROS 1 Noetic)

catkin-Paket, das ein oder mehrere BMS per Bluetooth LE ausliest und je BMS als
`sensor_msgs/BatteryState` publiziert – optional zusätzlich als `mower_msgs/Bms` für die
OpenMower-Logik. Verbindung und Protokoll übernimmt die Bibliothek
[aiobmsble](https://pypi.org/p/aiobmsble/) (patman15/BMS_BLE-HA, Apache-2.0); dieses Paket ist
ein dünner ROS-Wrapper darum.

Unterstützt: **JK-BMS** (z. B. JK-BD4A8S4P, JK02_32S) und **ANT-BMS** (`ANT-BLE…`), weitere
aiobmsble-Typen über den Modulnamen.

## Architektur

aiobmsble braucht **Python ≥ 3.12**, ROS Noetic läuft auf **Python 3.8**. Deshalb läuft die
Bibliothek in einem eigenen Prozess („Bridge“) mit eigener Python-Umgebung:

```
┌─────────── rospy-Node (Python 3.8) ────────────┐        ┌──── Bridge (Python 3.12) ─────┐
│ Hauptthread: Queue leeren → publizieren         │ stdin  │ bridge/bms_bridge.py           │
│   battery_state/<name>, /battery_state (primary)│ ─────► │  aiobmsble + bleak             │
│   battery_state/combined, /ll/bms, /diagnostics │ config │  je BMS ein asyncio-Task:      │
│                                                  │        │   Scan → Typ → async_update()  │
│ Supervisor-Thread: startet/überwacht Bridge,     │ stdout │   harte Timeouts, Reconnect,   │
│   liest JSON-Zeilen → queue.Queue               │ ◄───── │   Aufräumen hängender BlueZ-   │
└──────────────────────────────────────────────────┘ JSON   │   Verbindungen                 │
                                                             └────────────────────────────────┘
```

- Ein ausgefallenes BMS blockiert die anderen nicht (eigener Task je Gerät).
- Stirbt die Bridge, startet der Node sie nach `reconnect_interval_s` neu. Schließt der Node
  seine stdin-Pipe (Beenden, Absturz), trennt die Bridge alle BMS und beendet sich.
- BLE-Scans werden serialisiert (BlueZ erlaubt keine parallelen Scans). Gleichzeitige
  Verbindungen sind durch `max_connections` begrenzt, mit klarer Warnung, wenn ein Gerät warten muss.

| Datei | Inhalt |
|---|---|
| `bridge/bms_bridge.py` | Bridge: aiobmsble, Geräteerkennung, Reconnect, Timeouts |
| `bridge/setup_venv.sh`, `bridge/requirements.txt` | legt die Python-3.12-Umgebung an |
| `src/bms_ble/bridge_client.py` | startet/überwacht die Bridge, Thread → Queue |
| `src/bms_ble/config.py` | Validierung von `bms_list` |
| `src/bms_ble/model.py` | Datenmodell (aiobmsble-Werte → typisiert, fehlende Werte = `None`) |
| `src/bms_ble/battery_logic.py` | `power_supply_status` / `power_supply_health` |
| `src/bms_ble/conversion.py` | → `BatteryState`, combined, `mower_msgs/Bms` |
| `src/bms_ble/node.py` | ROS-Node |

## Installation (Raspberry Pi CM4, ROS Noetic)

```bash
# 1. Paket in den Workspace (z. B. den open_mower_ros-Workspace) und bauen
cd ~/catkin_ws/src
git clone https://github.com/kiwi220/BMS-Openmower.git bms_ble
cd .. && catkin_make && source devel/setup.bash

# 2. Python-3.12-Umgebung für die Bridge (nutzt uv, installiert es bei Bedarf;
#    lädt ein eigenständiges CPython, ändert das System-Python nicht)
src/bms_ble/bridge/setup_venv.sh            # -> ~/.local/share/bms_ble/venv

# 3. MACs finden. Die Hersteller-Apps vorher schließen: ein BMS akzeptiert
#    nur eine BLE-Verbindung gleichzeitig!
bluetoothctl --timeout 15 scan on           # "JK-BD4A8S-4P", "ANT-BLE…"

# 4. config/bms.yaml anpassen, starten
roslaunch bms_ble bms_ble.launch
rostopic echo /battery_state
```

Ohne uv geht es auch mit einem vorhandenen Python ≥ 3.12:
`python3.12 -m venv ~/.local/share/bms_ble/venv && ~/.local/share/bms_ble/venv/bin/pip install -r bridge/requirements.txt`.
**Nicht** mit `pip install --break-system-packages` ins System-Python von Noetic installieren – dort
(3.8) läuft aiobmsble nicht.

**OpenMower im Container:** Läuft ROS in einem Container, gehören Workspace-Build *und* die
Bridge-Umgebung in den Container, und der Container braucht Zugriff auf den D-Bus des Hosts
(z. B. `-v /run/dbus:/run/dbus:ro`), weil bleak über BlueZ auf dem Host arbeitet. Details hängen
vom Image ab.

## Konfiguration

```yaml
bms_list:
  - name: main_pack               # Topic battery_state/main_pack
    type: jk                      # jk | ant | ant_leg | ant_new | auto | <aiobmsble-Modul>
    mac: "AA:BB:CC:DD:EE:FF"
    cell_count: 7
    nominal_capacity_ah: 10.0
    ble_connect_password: ""      # optional
    primary: true                 # zusätzlich auf /battery_state
publish_rate_hz: 1.0
openmower_status_topic: ""        # z. B. "/ll/bms"
pack_connection: ""               # series | parallel -> battery_state/combined
```

| Parameter | Default | Beschreibung |
|---|---|---|
| `bms_list` | – (Pflicht) | Liste der BMS, siehe oben. Ein einzelnes BMS wird automatisch `primary`. |
| `publish_rate_hz` | `1.0` | Publish-Rate, zugleich Abfrageintervall der Bridge |
| `openmower_status_topic` | `""` | `mower_msgs/Bms` des primären BMS (siehe unten) |
| `pack_connection` | `""` | `series`/`parallel`: aggregiertes `battery_state/combined` (nur bei > 1 BMS) |
| `bridge_python` | `~/.local/share/bms_ble/venv/bin/python` | Interpreter der Bridge |
| `bridge_log_level` | `WARNING` | Log-Level von aiobmsble (wird nach rosout weitergeleitet) |
| `max_connections` | `4` | max. gleichzeitige BLE-Verbindungen des Adapters |
| `reconnect_interval_s` | `5.0` | Wartezeit bis zum nächsten Versuch |
| `scan_timeout_s` | `10.0` | Suchdauer je Versuch |
| `connect_timeout_s` | `45.0` | hartes Limit für Verbinden + erste Messung |
| `update_timeout_s` | `20.0` | hartes Limit je weiterer Messung |
| `stale_timeout_s` | `30.0` | danach `power_supply_health = UNKNOWN` |
| `publish_diagnostics` | `true` | `/diagnostics` |

Die alten Einzelparameter (`bms_mac_address`, `cell_count`, …) funktionieren weiter, wenn
`bms_list` fehlt (→ ein JK-BMS namens `main`).

**Typen:**
- `jk`: aiobmsble `jikong_bms`.
- `ant`: wählt anhand des Gerätenamens `ant_leg_bms` (`ANT-BLE[01]*`, `ANT-BLE22*`) oder
  `ant_bms` (`ANT?BLE24*`, `ANT?BLE3*`). Passt der Name auf keins (z. B. `ANT-BLEUB…`), gibt es
  eine Warnung und einen Versuch mit `ant_bms`; kommen keine Daten, `type: ant_leg` setzen.
- `ant_leg` / `ant_new`: ANT-Variante erzwingen.
- `auto`: Erkennung per aiobmsble über die Advertising-Daten; nicht erkannte `ANT…`-Namen
  werden wie `ant` behandelt, sonst Warnung und kein Verbindungsversuch.

## Nachrichtenbelegung `sensor_msgs/BatteryState`

| Feld | Quelle (aiobmsble) |
|---|---|
| `voltage`, `current` | `voltage`, `current` (+ Laden, − Entladen) |
| `percentage` | `battery_level` / 100 |
| `charge` | `cycle_charge` (Restladung Ah); fehlt sie: SoC × `capacity` |
| `capacity` | `design_capacity` vom BMS; fehlt sie: `nominal_capacity_ah` |
| `design_capacity` | `nominal_capacity_ah` aus der Konfiguration |
| `temperature` | JK: MOSFET-Temperatur; sonst höchste gemeldete Temperatur |
| `cell_voltage` | erste `cell_count` Zellen, fehlende = NaN |
| `present` | BLE verbunden |
| `location` | Name aus der Konfiguration |
| `serial_number` | aus der Geräteinfo (falls geliefert) |
| `power_supply_status` | `DISCHARGING` (I < −0,1 A) → `FULL` (SoC ≥ 100 %) → `CHARGING` (I > 0,1 A) → `NOT_CHARGING`; ohne Strom `UNKNOWN` |
| `power_supply_health` | siehe unten; nach > `stale_timeout_s` ohne Daten `UNKNOWN` |

Nicht gelieferte Werte werden nicht erfunden: Sie sind `NaN`. Externe Temperaturfühler, MOSFET-Zustände,
Fehlercode, Zyklen und SOH stehen auf `/diagnostics`.

**Health:**
- **JK:** `problem_code` ist die JK-Fehlerbitmaske → `DEAD` (Zell-/Pack-Unterspannung),
  `OVERVOLTAGE`, `OVERHEAT`, `COLD`, `UNSPEC_FAILURE`; rein informative Bits bleiben `GOOD`.
- **ANT:** aiobmsble setzt `problem_code` aus MOSFET-Statuscodes zusammen, deren Byte-Reihenfolge
  zwischen den ANT-Varianten nicht eindeutig dokumentiert ist. Jedes gemeldete Problem wird daher
  als `UNSPEC_FAILURE` gemeldet statt geraten; der Rohcode steht in `/diagnostics`.

**Verbindungsabbruch:** letzte Werte weiter publizieren, `present=False`, Warn-Log. Nach mehr als
30 s zusätzlich `power_supply_health = UNKNOWN`. Vor dem ersten Datensatz: NaN-Werte.

**combined** (nur mit `pack_connection` und > 1 BMS, erst wenn alle Daten geliefert haben):
seriell V = Summe, I = Mittel, Ah und SoC = Minimum, Zellen hintereinander; parallel V = Mittel,
I und Ah = Summe, SoC = Ladung/Kapazität. Health = schlechtestes Pack, `present` nur wenn alle verbunden.

## OpenMower-Anbindung

Geprüft in `open_mower_ros` (upstream `main`, Sep. 2026 – bitte gegen den installierten Stand
v1.2 gegenprüfen):

- `mower_logic` abonniert **`/ll/power`** (`mower_msgs/Power`) und **`/ll/bms`** (`mower_msgs/Bms`),
  **nicht** `sensor_msgs/BatteryState`.
- Spannung für Docking/kritische Schwelle:
  `GetFirstValid({power.battery_voltage_adc, bms.voltage, power.battery_voltage})` – die
  Mainboard-Messung hat Vorrang, `/ll/bms` greift nur, wenn diese fehlt.
- `/ll/power` gehört `mower_comms` (Ladespannung = Dock-Erkennung) → wird als Ziel **abgelehnt**.
- `openmower_status_topic: "/ll/bms"` publiziert das **primäre** BMS als `mower_msgs/Bms`. Vorher mit
  `rostopic info /ll/bms` prüfen, ob `mower_comms_v2` dort schon publiziert. Nach > 30 s ohne Daten
  gehen NaN-Werte raus, damit keine veraltete Spannung weiterverwendet wird.

## Passwort

- **JK:** aiobmsble sendet für JK kein Passwort (`accept_secret = False`); der Parameter wird mit
  einem Info-Log ignoriert. Nach allem, was bekannt ist, prüft nur die JK-App die PIN.
- **ANT (neue Variante, `ant_bms`):** aiobmsble sendet das Passwort als Auth-Kommando, wenn gesetzt.
- Beim ersten Hardwaretest prüfen, ob die Geräte ohne Passwort Daten liefern.

## Erste Inbetriebnahme – Checkliste

Tipp: Die BLE-Punkte lassen sich vorher am PC klären, siehe
[Hardwaretest am PC ohne ROS](#hardwaretest-am-pc-ohne-ros-windows-oder-linux).

- [ ] `bridge/setup_venv.sh` gibt „aiobmsble OK“ aus
- [ ] Log: „BMS bridge ready“, dann „BMS 'main_pack' connected (jikong_bms)“ und die Geräteinfo
- [ ] `/battery_state/main_pack` mit der JK-App vergleichen (Spannung, Strom-Vorzeichen, SoC)
- [ ] ANT: welches Modul wurde gewählt (Log „identified as“ bzw. Warnung „matches no … pattern“)?
      Liefert es Daten? Falls nicht: `type: ant_leg` bzw. `ant_new` probieren.
- [ ] Passwort-Frage klären (siehe oben)
- [ ] Ein BMS ausschalten: `present=False`, nach 30 s `UNKNOWN`, das andere läuft weiter;
      danach automatischer Reconnect
- [ ] Mit zwei BMS: klappen beide Verbindungen gleichzeitig (sonst `max_connections` senken und
      auf die Warnung achten)?
- [ ] Nur falls gewünscht: `openmower_status_topic: /ll/bms`, vorher `rostopic info /ll/bms`

Mehr Details aus aiobmsble: `bridge_log_level: DEBUG`.

## Tests

### Unit-Tests (ohne Hardware, ohne ROS, jedes Betriebssystem)

```bash
cd test
# ROS-Seite (Python 3.8 bzw. System-Python)
python3 -m unittest test_config test_battery_logic test_conversion test_bridge_client
# Bridge gegen das echte aiobmsble mit simuliertem BLE-Client und echten JK-Frames
~/.local/share/bms_ble/venv/bin/python -m unittest test_bridge_aiobmsble
```

### Hardwaretest am PC ohne ROS (Windows oder Linux)

Die Bridge ist ein normales Python-Programm und läuft auch ohne ROS. So lässt sich vor der
Installation auf dem Mäher klären, ob die BMS Daten liefern, ob ein Passwort nötig ist und welche
ANT-Variante erkannt wird.

Voraussetzungen: PC mit Bluetooth LE und Python ≥ 3.12. **macOS geht nicht**, weil macOS keine
MAC-Adressen herausgibt und die Bridge die Geräte über die MAC sucht.

**Windows (PowerShell):**

```powershell
git clone https://github.com/kiwi220/BMS-Openmower.git
cd BMS-Openmower
py -3.12 -m venv venv
venv\Scripts\pip install -r bridge\requirements.txt
venv\Scripts\python bridge\bms_bridge.py
```

**Linux:**

```bash
git clone https://github.com/kiwi220/BMS-Openmower.git && cd BMS-Openmower
bridge/setup_venv.sh ./venv
./venv/bin/python bridge/bms_bridge.py
```

Das Programm wartet danach auf **eine Zeile** mit der Konfiguration. Einfügen (MACs anpassen,
ein oder beide BMS) und Enter drücken:

```json
{"devices":[{"name":"jk","type":"jk","mac":"C8:47:80:XX:XX:XX"},{"name":"ant","type":"ant","mac":"AA:BB:CC:XX:XX:XX"}],"log_level":"INFO"}
```

Pro Gerät sind `name`, `type` (wie in `bms_list`), `mac` und optional `password` möglich. Die
Bridge gibt dann JSON-Zeilen aus:

| `event` | Bedeutung |
|---|---|
| `ready` | Bridge gestartet, mit aiobmsble- und bleak-Version |
| `log` | Meldungen, z. B. „connected to …“, „identified as …“, „not found“ |
| `state` | Verbindung auf- oder abgebaut |
| `device_info` | Modell, Firmware, Seriennummer |
| `sample` | Messwerte etwa jede Sekunde: `voltage`, `current`, `battery_level` (SoC), `cell_voltages`, `temp_values`, `chrg_mosfet`/`dischrg_mosfet`, `problem_code` |

**Beenden:** Windows Strg+Z, dann Enter; Linux Strg+D. Die Bridge trennt dabei alle BMS sauber.

Hinweise:
- Die Hersteller-Apps vorher schließen: Ein BMS erlaubt nur eine BLE-Verbindung gleichzeitig.
- MAC finden: unter Linux `bluetoothctl --timeout 15 scan on`, unter Windows z. B. mit der
  Handy-App „nRF Connect“ (Gerätename `JK-…` bzw. `ANT-BLE…`).
- Mehr Details: `"log_level":"DEBUG"` zeigt alle Bytes, die aiobmsble sendet und empfängt.
- Liefert ein ANT keine Daten: `"password":"1234"` im Geräteeintrag ergänzen bzw. `"type":"ant_leg"`
  oder `"type":"ant_new"` probieren. Die Einstellung, die funktioniert, genauso in `bms_list` übernehmen.
- Werte mit der App vergleichen, vor allem Spannung, Vorzeichen des Stroms (+ Laden, − Entladen) und SoC.

### Kompletter ROS-Node am PC

Nur unter Linux mit ROS Noetic (Ubuntu 20.04 oder ein `ros:noetic`-Docker-Container mit
`-v /run/dbus:/run/dbus`); Ablauf wie unter [Installation](#installation-raspberry-pi-cm4-ros-noetic).

## Referenzen

- [patman15/BMS_BLE-HA](https://github.com/patman15/BMS_BLE-HA) / [aiobmsble](https://pypi.org/p/aiobmsble/) (Apache-2.0)
- [syssi/esphome-jk-bms](https://github.com/syssi/esphome-jk-bms), [syssi/esphome-ant-bms](https://github.com/syssi/esphome-ant-bms) – Testframes in `test/fixtures.py`, JK-Fehlerbits
- [open_mower_ros](https://github.com/ClemensElflein/open_mower_ros) `mower_msgs`, `mower_logic`
