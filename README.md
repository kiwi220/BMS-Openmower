# bms_ble – BLE-Batteriemanagement (JK, JBD, ANT) für OpenMower (ROS 1 Noetic)

catkin-Paket, das ein oder mehrere BMS per Bluetooth LE ausliest und je BMS als
`sensor_msgs/BatteryState` publiziert – optional zusätzlich als `mower_msgs/Bms` für die
OpenMower-Logik. Verbindung und Protokoll übernimmt die Bibliothek
[aiobmsble](https://pypi.org/p/aiobmsble/) (patman15/BMS_BLE-HA, Apache-2.0); dieses Paket ist
ein dünner ROS-Wrapper darum.

Unterstützt: **JK-BMS** (z. B. JK-BD4A8S4P, JK02_32S), **JBD-BMS** (Jiabaida, `JBD-…`) und **ANT-BMS** (`ANT-BLE…`), weitere
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
| `src/bms_ble/xbot_sensors.py` | BMS-Werte als OpenMower-Sensoren (xbot_monitoring) |
| `src/bms_ble/node.py` | ROS-Node |

## Installation (Raspberry Pi CM4, ROS Noetic)

```bash
# 1. Paket in den open_mower_ros-Workspace legen und bauen. Nur dort gibt es
#    mower_msgs (/ll/bms) und xbot_msgs (OpenMower-Sensoren); in einem eigenen
#    Workspace läuft der Node auch, aber ohne diese beiden Funktionen.
cd ~/open_mower_ros/src                    # Pfad deines Workspaces
git clone https://github.com/kiwi220/BMS-Openmower.git bms_ble
cd .. && catkin_make                       # bzw. "catkin build bms_ble", je nachdem womit der Workspace gebaut ist
source devel/setup.bash
python3 -c "import xbot_msgs.msg, mower_msgs.msg; print('OpenMower-Messages OK')"

# 2. Python-3.12-Umgebung für die Bridge (nutzt uv, installiert es bei Bedarf;
#    lädt ein eigenständiges CPython, ändert das System-Python nicht)
src/bms_ble/bridge/setup_venv.sh            # -> ~/.local/share/bms_ble/venv

# 3. MACs finden. Die Hersteller-Apps vorher schließen: ein BMS akzeptiert
#    nur eine BLE-Verbindung gleichzeitig!
bluetoothctl --timeout 15 scan on           # "JK-BD4A8S-4P", "JBD-…", "ANT-BLE…"

# 4. config/bms.yaml anpassen (MACs, cell_count, Kapazität, primary), starten
roslaunch bms_ble bms_ble.launch
rostopic echo /battery_state

# 5. OpenMower-Sensoren prüfen (Standard: an, für das primäre BMS)
rostopic list | grep xbot_monitoring/sensors/bms_
```

Im Log sollten nach dem Start stehen:
- `BMS bridge ready (aiobmsble …, bleak …)`
- `BMS '<name>' connected (jikong_bms)` bzw. `(jbd_bms)` / `(ant_bms)` / `(ant_leg_bms)`
- `Publishing <N> xbot_monitoring sensors for <name> at 1.0 Hz`, nach den ersten Daten
  `xbot sensors added: bms_<name>_temp_mosfet, …`

Steht dort stattdessen `xbot sensors disabled: xbot_msgs is not importable`, ist der
open_mower_ros-Workspace beim Start nicht gesourct. Der Node läuft dann ohne die
OpenMower-Sensoren weiter.

**Aktualisieren** auf einen neueren Stand:

```bash
cd ~/open_mower_ros/src/bms_ble && git pull
cd ../.. && catkin_make && source devel/setup.bash    # bzw. catkin build bms_ble
src/bms_ble/bridge/setup_venv.sh                      # nur nötig, wenn sich bridge/requirements.txt geändert hat
```

Danach den Node bzw. OpenMower neu starten. Wer noch das alte Paket `jk_bms_ble` im Workspace hat:
den Ordner löschen, sonst baut catkin beide.

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
    type: jk                      # jk | jbd | ant | ant_leg | ant_new | auto | <aiobmsble-Modul>
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
| `max_connections` | `4` | max. gleichzeitige BLE-Verbindungen des Adapters (ganze Zahl ≥ 1). Zeit- und Intervall-Parameter müssen > 0 sein, sonst startet der Node nicht und nennt den Parameter |
| `reconnect_interval_s` | `5.0` | Wartezeit bis zum nächsten Versuch |
| `scan_timeout_s` | `10.0` | Suchdauer je Versuch |
| `connect_timeout_s` | `45.0` | hartes Limit für Verbinden + erste Messung |
| `update_timeout_s` | `20.0` | hartes Limit je weiterer Messung |
| `stale_timeout_s` | `30.0` | danach `power_supply_health = UNKNOWN` |
| `publish_diagnostics` | `true` | `/diagnostics` |
| `publish_xbot_sensors` | `true` | BMS-Werte als OpenMower-Sensoren, siehe [OpenMower-Sensoren / MowBite](#openmower-sensoren--mowbite) |
| `xbot_sensors_rate_hz` | `1.0` | Rate der Sensorwerte, höchstens `2.0` |
| `xbot_sensor_bms` | `""` | `""` = primäres BMS, `all` = alle, oder ein Name aus `bms_list` |

Die alten Einzelparameter (`bms_mac_address`, `cell_count`, …) funktionieren weiter, wenn
`bms_list` fehlt (→ ein JK-BMS namens `main`).

**Typen:**
- `jk`: aiobmsble `jikong_bms`.
- `jbd`: aiobmsble `jbd_bms`. aiobmsble erkennt JBD nur an bestimmten Namen (`JBD-*` und einige
  OEM-Namen) und an bestimmten MAC-Adress-Präfixen. Meldet sich dein Akku anders, steht bei `type: auto` eine
  Warnung („not recognized“) und, wenn er den JBD-Dienst `ff00` anbietet, der Hinweis „try type: jbd“.
  Mit `type: jbd` wird der Name nicht geprüft.
- `ant`: wählt anhand des Gerätenamens `ant_leg_bms` (`ANT-BLE[01]*`, `ANT-BLE22*`) oder
  `ant_bms` (`ANT?BLE24*`, `ANT?BLE3*`). Passt der Name auf keins (z. B. `ANT-BLEUB…`), gibt es
  eine Warnung und einen Versuch mit `ant_bms`; kommen keine Daten, `type: ant_leg` setzen.
- `ant_leg` / `ant_new`: ANT-Variante erzwingen.
- `auto`: Erkennung per aiobmsble über die Advertising-Daten; nicht erkannte `ANT…`-Namen
  werden wie `ant` behandelt, sonst Warnung und kein Verbindungsversuch. Für den Dauerbetrieb
  ist ein fester Typ robuster.

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
- **JBD:** `problem_code` ist der Schutzstatus des JBD (Bits 0–13, Namen aus esphome-jbd-bms):
  Zell-/Pack-Überspannung → `OVERVOLTAGE`, Zell-/Pack-Unterspannung → `DEAD`, Lade-/Entlade-
  Übertemperatur → `OVERHEAT`, Untertemperatur → `COLD`. Jedes andere gesetzte Bit (Überstrom,
  Kurzschluss, IC-Fehler, MOSFET-Software-Sperre, Ladezeit-Timeout, undokumentierte Bits) wird als
  `UNSPEC_FAILURE` gemeldet statt ignoriert: Beim JBD bedeutet jedes Bit einen aktiven Schutz.
  **Ausnahme Ladeende:** Steht der Zellüberspannungsschutz bei **100 % SoC** an, ist das Laden über
  diesen Schutz beendet worden (der echte Mitschnitt in `test/fixtures.py` zeigt genau das: 100 %,
  Schutz aktiv, Lade-MOSFET aus). Dann bleibt die Health `GOOD`, wie beim JK-Bit „Battery is fully
  charged“. Das Bit wird weiter angezeigt: in `/diagnostics` als Warnung, im xbot-Status als
  `Full, Cell overvoltage`, in `/ll/bms` unter `battery_status`. Unter 100 % oder ohne SoC-Wert bleibt
  es `OVERVOLTAGE` (dann hat eine Zelle die Schutzgrenze erreicht, obwohl der Akku nicht als voll gilt). Pack-Überspannung (Bit 2) ist immer
  `OVERVOLTAGE`.
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

## OpenMower-Sensoren / MowBite

Zusätzlich zu `BatteryState` kann der Node die BMS-Werte als **xbot_monitoring-Sensoren**
veröffentlichen. OpenMowers `xbot_monitoring` findet sie selbst (es sucht im ROS-Master nach
`/xbot_monitoring/sensors/.*/info`) und reicht sie per MQTT weiter (`sensor_infos/json`,
`sensors/<id>/data`). [MowBite](https://github.com/mkaaaaaay/mowbite) und andere MQTT-Clients
zeigen sie dann an, ohne dass an OpenMower oder MowBite etwas geändert wird.

Je Sensor gibt es zwei Topics: `/xbot_monitoring/sensors/<id>/info` (`xbot_msgs/SensorInfo`,
latched) und `/xbot_monitoring/sensors/<id>/data` (`xbot_msgs/SensorDataDouble` bzw.
`SensorDataString`). `xbot_msgs` kommt aus dem `open_mower_ros`-Workspace und ist optional: Fehlt
es, gibt der Node eine Warnung aus und läuft ohne diese Sensoren weiter.

| Parameter | Default | Beschreibung |
|---|---|---|
| `publish_xbot_sensors` | `true` | Feature ein/aus |
| `xbot_sensors_rate_hz` | `1.0` | Rate der Sensorwerte, höchstens `2.0` (OpenMower drosselt seine eigenen Sensoren ebenfalls auf 2 Hz). Kann nicht schneller sein als `publish_rate_hz`. Gesendet wird nach festem Zeitplan mit 10 % Toleranz, damit kleine Zeitschwankungen der Schleife keine Durchläufe verwerfen; im Mittel wird die Rate nicht überschritten. |
| `xbot_sensor_bms` | `""` | `""` = das primäre BMS (ist keins primär: Feature aus, mit Warnung), `all` = alle BMS, oder ein Name aus `bms_list` |

**Sensoren je BMS.** ID-Schema `bms_<name>_<messwert>`, `<name>` in Kleinbuchstaben und nur
`[a-z0-9_]`. Der Anzeigename ist `BMS <name> <Messwert>`, bei nur einem veröffentlichten BMS
`BMS <Messwert>`. IDs mit `om_` (OpenMowers eigene Sensoren) entstehen nie.

| ID-Suffix | Typ | Beschreibung | Einheit | Quelle |
|---|---|---|---|---|
| `voltage` | DOUBLE | VOLTAGE | V | Gesamtspannung |
| `current` | DOUBLE | CURRENT | A | Strom (+ Laden, − Entladen) |
| `soc` | DOUBLE | PERCENT | % | SoC des BMS (0–100) |
| `temp_mosfet` | DOUBLE | TEMPERATURE | deg.C | MOSFET-Temperatur, nur wenn geliefert |
| `temp_1`, `temp_2`, … | DOUBLE | TEMPERATURE | deg.C | externe Fühler (JK: T1/T2, JBD: NTC-Fühler); nicht angeschlossene lässt aiobmsble weg |
| `cell_01` … `cell_NN` | DOUBLE | VOLTAGE | V | Zellspannungen, Anzahl = `cell_count` |
| `cell_delta` | DOUBLE | VOLTAGE | V | max − min der gelieferten Zellen (innerhalb `cell_count`, ab 2 Zellen) |
| `status` | STRING | UNKNOWN | – | `OK`, `Charging`, `Discharging`, `Full` plus Fehlernamen (JK, JBD) bzw. `problem code 0x…` (ANT); `Disconnected` / `Stale` bei Ausfall |

aiobmsble meldet beim JBD alle Fühler als Zelltemperaturen und keine MOSFET-Temperatur: Dort gibt es
kein `temp_mosfet`, und die `temperature` im `BatteryState` ist die höchste Fühlertemperatur.

Die Temperatursensoren entstehen erst mit dem ersten Datensatz, weil erst dann feststeht, welche
Fühler das BMS liefert. Ihre Info wird dann latched nachveröffentlicht, und `xbot_monitoring`
nimmt sie beim nächsten Suchlauf auf.

**Grenzwerte:** Gesetzt ist nur ein Anzeigebereich für `soc` (0–100 %) und die Zellspannungen
(2,5–4,2 V, deckt Li-Ion und LiFePO4 ab). Kritische Grenzen sind bewusst nicht gesetzt
(`has_critical_* = False`, Wert `-1`), weil sie Sicherheitsaussagen zur konkreten Zellchemie und
BMS-Einstellung sind. Sie ließen sich später als Parameter ergänzen.

**Datenverhalten:** Gesendet wird nur, was der Datensatz wirklich enthält (fehlend/NaN = kein
Wert, nie `0.0`). Ist das BMS getrennt oder liefert länger als `stale_timeout_s` keine Daten, gehen
**keine Messwerte** mehr raus; nur `status` meldet `Disconnected` bzw. `Stale`. **Achtung:** MowBite
und andere MQTT-Clients zeigen dann weiter den zuletzt empfangenen Wert an. Maßgeblich ist in
diesem Fall der `status`-Sensor.

**Wichtig zur Akku-Anzeige:** Die große Akku-Anzeige im MowBite-Dashboard kommt weiterhin aus
OpenMowers eigenem, spannungsbasiertem Prozentwert (`om_v_battery` usw.). Der SoC des BMS erscheint
nur als zusätzlicher Sensor `bms_<name>_soc`.

**Prüfen auf dem Mäher:**

```bash
rostopic list | grep xbot_monitoring/sensors/bms_
rostopic echo -n1 /xbot_monitoring/sensors/bms_main_pack_voltage/info
mosquitto_sub -h <mäher> -t 'sensor_infos/json' -C 1
mosquitto_sub -h <mäher> -t 'sensors/bms_main_pack_voltage/data'
```

**Teststand:** Ohne echten Mäher geprüft, und zwar per Unit-Tests (ID-Schema, Sensorliste,
fehlende Werte, Stale/Disconnected, Zell-Delta, eindeutige IDs mit zwei BMS). Zusätzlich
serialisieren die Tests alle Nachrichten mit den echten, aus `open_mower_ros` erzeugten
`xbot_msgs`-Klassen, und ein Durchlauf mit echter Bridge und simuliertem BLE hat
Registrierung, Nachregistrierung der Temperaturen und das Verhalten beim Verbindungsabbruch
gezeigt. Ob `xbot_monitoring` und MowBite die Sensoren wie erwartet anzeigen, ist noch nicht auf
einem Mäher getestet.

## Passwort

- **JK:** aiobmsble sendet für JK kein Passwort (`accept_secret = False`); der Parameter wird mit
  einem Info-Log ignoriert. Nach allem, was bekannt ist, prüft nur die JK-App die PIN.
- **JBD:** aiobmsble sendet das Passwort als Anmelde-Kommando, wenn eins gesetzt ist. Ein **falsches
  Passwort lässt die Verbindung scheitern** (Log: `connect failed … PermissionError`), das BMS
  gilt dann nie als verbunden. Ohne Passwort wird nichts gesendet. Ob dein JBD ein Passwort
  verlangt, ist nicht geprüft: Zuerst ohne Passwort versuchen und nur eintragen, wenn die
  Hersteller-App eins verlangt.
- **ANT (neue Variante, `ant_bms`):** aiobmsble sendet das Passwort als Auth-Kommando, wenn gesetzt.
- Beim ersten Hardwaretest prüfen, ob die Geräte ohne Passwort Daten liefern.

## Erste Inbetriebnahme – Checkliste

Tipp: Die BLE-Punkte lassen sich vorher am PC klären, siehe
[Hardwaretest am PC ohne ROS](#hardwaretest-am-pc-ohne-ros-windows-oder-linux).

- [ ] `bridge/setup_venv.sh` gibt „aiobmsble OK“ aus
- [ ] Log: „BMS bridge ready“, dann „BMS 'main_pack' connected (jikong_bms)“ und die Geräteinfo
- [ ] `/battery_state/main_pack` mit der JK-App vergleichen (Spannung, Strom-Vorzeichen, SoC)
- [ ] JBD: Meldet sich der Akku mit einem Namen, den aiobmsble kennt? Sonst `type: jbd` fest setzen.
      Stimmen Zellspannungen und Fühlertemperaturen mit der JBD-App überein?
- [ ] ANT (falls vorhanden): welches Modul wurde gewählt (Log „identified as“ bzw. Warnung
      „matches no … pattern“)? Liefert es Daten? Falls nicht: `type: ant_leg` bzw. `ant_new` probieren.
- [ ] Passwort-Frage klären (siehe oben)
- [ ] Ein BMS ausschalten: `present=False`, nach 30 s `UNKNOWN`, das andere läuft weiter;
      danach automatischer Reconnect
- [ ] Mit zwei BMS: klappen beide Verbindungen gleichzeitig (sonst `max_connections` senken und
      auf die Warnung achten)?
- [ ] Nur falls gewünscht: `openmower_status_topic: /ll/bms`, vorher `rostopic info /ll/bms`
- [ ] OpenMower-Sensoren: Prüfbefehle aus [OpenMower-Sensoren / MowBite](#openmower-sensoren--mowbite); erscheinen die `bms_…`-Sensoren in MowBite?

Mehr Details aus aiobmsble: `bridge_log_level: DEBUG`.

## Tests

### Unit-Tests (ohne Hardware, ohne ROS, jedes Betriebssystem)

```bash
cd test
# ROS-Seite (Python 3.8 bzw. System-Python)
python3 -m unittest test_config test_battery_logic test_conversion test_bridge_client test_xbot_sensors
# Bridge gegen das echte aiobmsble mit simuliertem BLE-Client und echten JK-Frames
~/.local/share/bms_ble/venv/bin/python -m unittest test_bridge_aiobmsble
```

| Testdatei | Prüft |
|---|---|
| `test_config.py` | `bms_list`-Validierung, alte Einzelparameter, `pack_connection` |
| `test_battery_logic.py` | `power_supply_status` / `power_supply_health`, JK-Fehlerbits |
| `test_conversion.py` | aiobmsble-Daten → `BatteryState`, combined, `mower_msgs/Bms` |
| `test_bridge_client.py` | Start, Absturz und Neustart der Bridge |
| `test_xbot_sensors.py` | OpenMower-Sensoren: ID-Schema, Sensorliste, fehlende Werte, Stale/Disconnected, Zell-Delta, eindeutige IDs mit zwei BMS, Parameter |
| `test_bridge_aiobmsble.py` | Bridge gegen das echte aiobmsble (nur mit Python ≥ 3.12) |

Ohne ROS werden einige Tests übersprungen (`skipped`): die Abgleiche mit den echten Nachrichten
`sensor_msgs/BatteryState` und `xbot_msgs/SensorInfo` sowie unter Python 3.8 die aiobmsble-Tests.

**Mit den echten ROS-Nachrichten** (auf dem Mäher oder einem Noetic-Rechner, Workspace gesourct)
laufen die Abgleiche mit:

```bash
source ~/open_mower_ros/devel/setup.bash
cd ~/open_mower_ros/src/bms_ble/test
python3 -m unittest test_config test_battery_logic test_conversion test_bridge_client test_xbot_sensors
# oder über catkin (nosetests):
cd ~/open_mower_ros && catkin_make run_tests_bms_ble && catkin_test_results build/test_results/bms_ble
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
{"devices":[{"name":"jk","type":"jk","mac":"C8:47:80:XX:XX:XX"},{"name":"jbd","type":"jbd","mac":"A5:C2:37:XX:XX:XX"}],"log_level":"INFO"}
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
  Handy-App „nRF Connect“ (Gerätename `JK-…`, `JBD-…` bzw. `ANT-BLE…`).
- Mehr Details: `"log_level":"DEBUG"` zeigt alle Bytes, die aiobmsble sendet und empfängt.
- Verbindet sich ein JBD nicht und im Log steht `PermissionError` / „incorrect secret“, ist das
  Passwort falsch oder gar nicht erwartet: `"password"` weglassen oder korrigieren.
- Liefert ein ANT keine Daten: `"password":"1234"` im Geräteeintrag ergänzen bzw. `"type":"ant_leg"`
  oder `"type":"ant_new"` probieren. Die Einstellung, die funktioniert, genauso in `bms_list` übernehmen.
- Werte mit der App vergleichen, vor allem Spannung, Vorzeichen des Stroms (+ Laden, − Entladen) und SoC.

### Kompletter ROS-Node am PC

Nur unter Linux mit ROS Noetic (Ubuntu 20.04 oder ein `ros:noetic`-Docker-Container mit
`-v /run/dbus:/run/dbus`); Ablauf wie unter [Installation](#installation-raspberry-pi-cm4-ros-noetic).
Ohne open_mower_ros-Workspace fehlen `mower_msgs` und `xbot_msgs`: Der Node läuft, meldet aber
`xbot sensors disabled` und veröffentlicht nur `BatteryState` und `/diagnostics`.

### Test auf dem Mäher (Schritt für Schritt)

1. **Node läuft, BMS verbunden:** Log-Zeilen wie unter
   [Installation](#installation-raspberry-pi-cm4-ros-noetic) Schritt 5.
2. **BatteryState:** `rostopic echo -n1 /battery_state` – Spannung, Strom-Vorzeichen und SoC mit
   der Hersteller-App vergleichen; `present: True`.
3. **OpenMower-Sensoren auf ROS-Ebene:**
   ```bash
   rostopic list | grep xbot_monitoring/sensors/bms_
   rostopic echo -n1 /xbot_monitoring/sensors/bms_main_pack_voltage/info    # sensor_name, unit "V", value_type 2
   rostopic echo -n1 /xbot_monitoring/sensors/bms_main_pack_voltage/data
   rostopic echo -n1 /xbot_monitoring/sensors/bms_main_pack_status/data     # "OK", "Charging", ...
   ```
   `bms_main_pack` durch `bms_<dein name in Kleinbuchstaben>` ersetzen.
4. **MQTT** (`sudo apt install mosquitto-clients`, auf dem Mäher oder einem Rechner im selben Netz):
   ```bash
   mosquitto_sub -h <mäher> -t 'sensor_infos/json' -C 1 | grep -o '"bms_[a-z0-9_]*"' | sort -u
   mosquitto_sub -h <mäher> -t 'sensors/bms_main_pack_voltage/data'
   ```
   Das gilt für den lokalen Broker des Mähers (Port 1883). An einen externen Broker
   (`OM_MQTT_ENABLE`) schickt `xbot_monitoring` dieselben Topics mit `OM_MQTT_TOPIC_PREFIX` davor.
5. **MowBite:** In der Sensor-Ansicht sollten die `BMS …`-Sensoren erscheinen, Temperaturen in °C.
6. **Ausfall:** BMS ausschalten oder außer Reichweite bringen. Erwartet:
   - `/battery_state`: `present: False`, nach `stale_timeout_s` (30 s) `power_supply_health: 0` (UNKNOWN)
   - `bms_…_status`: `Disconnected`, danach keine neuen Werte auf den übrigen `bms_…`-Sensoren
     (MowBite zeigt dort weiter den letzten Wert)
   - nach dem Wiedereinschalten automatischer Reconnect und wieder `OK`

## Lizenz

Dieses Paket steht unter der **GNU General Public License v3.0** (`GPL-3.0-only`), siehe
[`LICENSE`](LICENSE).

- [aiobmsble](https://pypi.org/p/aiobmsble/) (Apache-2.0) wird als eigenständige Bibliothek in
  einem separaten Prozess (der Bridge) verwendet und nicht mitgeliefert; `bridge/setup_venv.sh`
  installiert es aus PyPI.
- Die Testframes in `test/fixtures.py` stammen aus
  [syssi/esphome-jk-bms](https://github.com/syssi/esphome-jk-bms) und
  [syssi/esphome-jbd-bms](https://github.com/syssi/esphome-jbd-bms) (jeweils Apache-2.0); der
  Herkunftshinweis steht in der Datei.

## Referenzen

- [patman15/BMS_BLE-HA](https://github.com/patman15/BMS_BLE-HA) / [aiobmsble](https://pypi.org/p/aiobmsble/) (Apache-2.0)
- [syssi/esphome-jk-bms](https://github.com/syssi/esphome-jk-bms), [syssi/esphome-jbd-bms](https://github.com/syssi/esphome-jbd-bms), [syssi/esphome-ant-bms](https://github.com/syssi/esphome-ant-bms) – Testframes in `test/fixtures.py`, JK- und JBD-Fehlerbits
- [open_mower_ros](https://github.com/ClemensElflein/open_mower_ros) `mower_msgs`, `mower_logic`
