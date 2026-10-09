"""Real JK-BMS BLE frames used as test fixtures.

Taken from the syssi/esphome-jk-bms test suite
(tests/components/jk_bms_ble/frames_*.h, Apache-2.0), where they are
annotated with the values the reference decoder produces.
"""

DEVICE_INFO_JK02_32S_V11 = bytes.fromhex(
    "55aaeb90036b4a4b5f5042324131365331355000000031342e584100000031342e323000"
    "000088a30100940000004a4b5f5042324131365331355000000031323334000000000000"
    "000000000000323331313138000033303932353732313334003030303000496e70757420"
    "5573657264617461000031323334353700000000000000000000496e7075742055736572"
    "646174610000feffffffafe9010200000000901f00000000c0d8e7fe1f00000100000000"
    "000000000104cf030000000000000000000000000000df07000000000000000000000000"
    "00000b000000000000000000000000000000000b00010000000000000000090000000b00"
    "00000000000000000000805100000a5001000000000000000000000000000000000000fe"
    "9fe9fe0300000000000000de"
)

CELL_INFO_JK02_32S_V11 = bytes.fromhex(
    "55aaeb9002e8ae0c9e0c9a0c9f0ca10c9f0ca00ca00c990ca00c900c990ca50c9f0c990c"
    "aa0c0000000000000000000000000000000000000000000000000000000000000000ffff"
    "00009f0c1f00000a680068007a007300720085007000670082007700650066007e007800"
    "74009c000000000000000000000000000000000000000000000000000000000000000000"
    "ad0000000000e9c900000000000000000000b100b100000000000000003413040000d007"
    "000000000000000000006400000098a3010001010000000000000000000000000000ff00"
    "01000000e20400000100f6c74040000000003014fe010001010100060000600c00000000"
    "0000ad00b300b4009003da269d07180600008051010000000000000000000000000000fe"
    "ff7fdd2f0101b00700000016"
)

CELL_INFO_JK02_32S_V15 = bytes.fromhex(
    "55aaeb9002ac050dfe0cfe0c010d010dfd0cfb0c010dfc0cfb0cfe0cfb0cf80cfb0cfb0c"
    "090d0000000000000000000000000000000000000000000000000000000000000000ffff"
    "0000ff0c10000f0c40003d0040003d0041003f0041003e0041003e0041003d0040003e00"
    "41003f000000000000000000000000000000000000000000000000000000000000000000"
    "810000000000e8cf00004ae41900897c000086008000000000000000001986c00000400d"
    "030009000000b15f1c00640000008c4c760101010000000000000000000000000000ff00"
    "01000000050452000000d44e404000000000ca1400000001010100060000ea6700000000"
    "0000810083008700cc03b799da09160000008051010000000301000000000000000000fe"
    "ff7fdc2f0101b0cf070000d8"
)

DEVICE_INFO_JK02_24S_V10 = bytes.fromhex(
    "55aaeb9003fc4a4b2d4232413234533230500000000031302e584700000031302e303700"
    "00002c1b0400010000004a4b2d4232413234533230500000000031323334000000000000"
    "000000000000323230353131000032303431383033303238003030303000496e70757420"
    "557365726461746100003132333435360000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000fc"
)

CELL_INFO_JK02_24S_V10 = bytes.fromhex(
    "55aaeb9002fcd40cd00ccf0cd00cd00ccf0ccf0cd10cd00ccf0cd00ccf0cd20cd40ccf0c"
    "cf0cd00ccf0cd10cd20cd20cd20cd50cd10cffffff00d10c0500000364006c0068006300"
    "5f005d005b005e007f0079006b006f006f006800640060005b00580058005b0060006500"
    "680061000000000000008f3301000000000000000000cf00cf00ea0000000000005534a7"
    "000050c3000000000000f9340000640040051c1c04000100850600000000000000000000"
    "0000070001000000fa030000000000d23f4000000000e204000000000001000300001a19"
    "290000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000043"
)

# aiobmsble 0.29.0 async_update() output for CELL_INFO_JK02_32S_V11, as sent
# by bridge/bms_bridge.py (captured with test_bridge_aiobmsble fakes).
JK_SAMPLE_V11 = {"voltage": 51.689, "current": 0.0, "problem_code": 0, "balance_current": 0.0, "balancer": False, "battery_level": 52, "cycle_charge": 1.043, "design_capacity": 2, "cycles": 0, "battery_health": 100, "chrg_mosfet": True, "dischrg_mosfet": True, "temp_sensors": 255, "cell_count": 16, "delta_voltage": 0.031, "temp_values": [{"value": 17.3, "type": "MOSFET"}, {"value": 17.7, "type": "GENERIC"}, {"value": 17.7, "type": "GENERIC"}, {"value": 17.3, "type": "MOSFET"}, {"value": 17.9, "type": "GENERIC"}, {"value": 18.0, "type": "GENERIC"}], "cell_voltages": [3.246, 3.23, 3.226, 3.231, 3.233, 3.231, 3.232, 3.232, 3.225, 3.232, 3.216, 3.225, 3.237, 3.231, 3.225, 3.242], "battery_charging": False, "temperature": 17.65, "cycle_capacity": 53.912, "power": 0.0, "problem": False}


# --- JBD (Xiaoxiang / Jiabaida) -------------------------------------------
# From the syssi/esphome-jbd-bms test suite (tests/components/jbd_bms_ble/frames.h,
# Apache-2.0), where the decoded values below are annotated.

# Basic info (command 0x03): 4S, 5 Ah, 15.60 V, 0.00 A, 100 %, remaining 4.98 Ah,
# 3 temperature sensors 22.4 / 22.3 / 21.7 degC, MOSFETs charge+discharge on, no protection.
JBD_BASIC_INFO_4S = bytes.fromhex(
    "dd03001d0618000001f201f400002c7c00000000000080640304030b8b0b8a0b84fa8d77"
)

# Cell voltages (command 0x04): 3.909 / 3.901 / 3.895 / 3.901 V
JBD_CELL_INFO_4S = bytes.fromhex(
    "dd0400080f450f3d0f370f3dfec677"
)

# Basic info of a real hardware capture: 4S, 280 Ah, 14.28 V, 100 %, 8 cycles,
# protection status 0x0001 = cell overvoltage, only the discharge MOSFET on.
# The source only lists the payload; the frame header (dd 03 00 1d), CRC and
# end byte were added here (the same wrapping reproduces JBD_BASIC_INFO_4S).
JBD_BASIC_INFO_CELL_OVERVOLTAGE = bytes.fromhex(
    "dd03001d059400006d606d6000082c7c00000000000180640204030b820b7c0b77fa7c77"
)

# Hardware version (command 0x05): "JBD-SP04S034-L4S-200A-B-U" (framed like above).
JBD_HW_VERSION = bytes.fromhex(
    "dd0500194a42442d53503034533033342d4c34532d323030412d422d55fa0877"
)

# aiobmsble 0.29.0 async_update() output for the JBD frames above, as sent by
# bridge/bms_bridge.py (captured with the fakes from test_bridge_aiobmsble).
JBD_SAMPLE_4S = {'balancer': 0,
 'battery_charging': False,
 'battery_level': 100,
 'cell_count': 4,
 'cell_voltages': [3.909, 3.901, 3.895, 3.901],
 'chrg_mosfet': True,
 'current': 0.0,
 'cycle_capacity': 77.688,
 'cycle_charge': 4.98,
 'cycles': 0,
 'delta_voltage': 0.014,
 'design_capacity': 5,
 'dischrg_mosfet': True,
 'power': 0.0,
 'problem': False,
 'problem_code': 0,
 'temp_sensors': 3,
 'temp_values': [{'type': 'CELL', 'value': 22.4},
                 {'type': 'CELL', 'value': 22.3},
                 {'type': 'CELL', 'value': 21.7}],
 'temperature': 22.133,
 'voltage': 15.6}

JBD_SAMPLE_CELL_OVERVOLTAGE = {'balancer': 0,
 'battery_charging': False,
 'battery_level': 100,
 'cell_count': 4,
 'cell_voltages': [3.909, 3.901, 3.895, 3.901],
 'chrg_mosfet': False,
 'current': 0.0,
 'cycle_capacity': 3998.4,
 'cycle_charge': 280.0,
 'cycles': 8,
 'delta_voltage': 0.014,
 'design_capacity': 280,
 'dischrg_mosfet': True,
 'power': 0.0,
 'problem': True,
 'problem_code': 1,
 'temp_sensors': 3,
 'temp_values': [{'type': 'CELL', 'value': 21.5},
                 {'type': 'CELL', 'value': 20.9},
                 {'type': 'CELL', 'value': 20.4}],
 'temperature': 20.933,
 'voltage': 14.28}
