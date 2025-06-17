This repo contains the working codes that run the telemetry system of Agnirath (IITM's Solar Race Car)

The telemetry system is responsible for reliably collecting the sensor data of the car and relaying it to a chase vehicle roughly 1km behind  
It contains:
1. The LoRa code
    - These are to be uploaded onto an ESP32 (any arduino style microcontroller works too)
    - The Transmitter is on the car connected to a E32-900T30D module (seperate branch contains the AI-433 module codes) and to the onboard computer
    - An identical hardware setup is present on the Receiver side, with the Reciever codes instead

2. The ROS / Python codes:
    - This consists of the Uplink node on the onboard computer which compacts the data, formats it to packets as mentioned in the packet_structure.json file
    - The packets sent have the following structure: [header] [crc] [length] [type] [data]
    - The downlink consists of a python file which unpacks the data and sends it out as a dictionary of sensor values
