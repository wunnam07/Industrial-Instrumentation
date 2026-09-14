from pymodbus.client import ModbusTcpClient
import time

client = ModbusTcpClient("127.0.0.1", port=1502)

if client.connect():

    print("Connected")

    heartbeat = 1

    # Request heater ON
    client.write_coil(
        address=0,
        value=True
    )

    print("HEATER_CMD = ON")

    # Keep PLC heartbeat alive for 10 seconds
    for i in range(40):

        client.write_register(
            address=0,
            value=heartbeat
        )

        status = client.read_discrete_inputs(
            address=5,
            count=3
        )

        if not status.isError():
            print(
                "Heartbeat:", heartbeat,
                "| Permissive:", status.bits[0],
                "| Heater Output:", status.bits[1],
                "| PLC Comms:", status.bits[2]
            )

        heartbeat += 1
        time.sleep(1)

    print("Stopping PLC heartbeat...")
    print("HEATER_CMD is being left ON deliberately.")

    time.sleep(5)

    status = client.read_discrete_inputs(
        address=5,
        count=3
    )

    if not status.isError():
        print(
            "After heartbeat loss:",
            "| Permissive:", status.bits[0],
            "| Heater Output:", status.bits[1],
            "| PLC Comms:", status.bits[2]
        )

    client.close()

else:
    print("Could not connect")