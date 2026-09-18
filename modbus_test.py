from pymodbus.client import ModbusTcpClient
import time

client = ModbusTcpClient("127.0.0.1", port=1502)

if client.connect():
    print("Connected")

    previous_heartbeat = None

    # The simulated ESP32 is an instrumentation node. It publishes three
    # temperatures, a non-authoritative comparator candidate, diagnostics,
    # and its heartbeat; the PLC owns all simulated heater decisions.
    for _ in range(20):
        registers = client.read_input_registers(
            address=0,
            count=19,
        )
        status = client.read_discrete_inputs(
            address=0,
            count=12,
        )

        if not registers.isError() and not status.isError():
            heartbeat = registers.registers[8]
            heartbeat_changed = (
                previous_heartbeat is None
                or heartbeat != previous_heartbeat
            )
            print(
                "T1:", registers.registers[0],
                "| T2:", registers.registers[1],
                "| T3:", registers.registers[9],
                "| Diagnostic candidate:", registers.registers[2],
                "| Diagnostic:", registers.registers[6],
                "| ESP32 heartbeat:", heartbeat,
                "| Changed:", heartbeat_changed,
                "| Node ready:", status.bits[0],
            )
            previous_heartbeat = heartbeat

        time.sleep(0.5)

    client.close()

else:
    print("Could not connect")
