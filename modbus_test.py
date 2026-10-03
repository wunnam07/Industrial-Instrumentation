import time

from pymodbus.client import ModbusTcpClient

HOST = "127.0.0.1"
PORT = 1502
DEVICE_ID = 1

IR_TEMP1 = 0
IR_TEMP2 = 1
IR_TEMP3 = 2
IR_ESP32_HEARTBEAT = 3
IR_SENSOR1_HEALTH = 4
IR_SENSOR2_HEALTH = 5
IR_SENSOR3_HEALTH = 6

INVALID = 65535

client = ModbusTcpClient(HOST, port=PORT)

if client.connect():
    print("Connected to Virtual Modbus Interface v2")
    previous_heartbeat = None

    for _ in range(20):
        response = client.read_input_registers(
            address=0,
            count=7,
            device_id=DEVICE_ID,
        )

        if not response.isError():
            registers = response.registers
            heartbeat = registers[IR_ESP32_HEARTBEAT]
            heartbeat_changed = (
                previous_heartbeat is None
                or heartbeat != previous_heartbeat
            )

            print(
                "T1:", registers[IR_TEMP1],
                "| T2:", registers[IR_TEMP2],
                "| T3:", registers[IR_TEMP3],
                "| Health:",
                registers[IR_SENSOR1_HEALTH],
                registers[IR_SENSOR2_HEALTH],
                registers[IR_SENSOR3_HEALTH],
                "| Heartbeat:", heartbeat,
                "| Changed:", heartbeat_changed,
            )

            if all(
                registers[index] != INVALID
                for index in (IR_TEMP1, IR_TEMP2, IR_TEMP3)
            ):
                print("All three published temperatures are currently valid")

            previous_heartbeat = heartbeat

        time.sleep(0.5)

    client.close()
else:
    print("Could not connect")
