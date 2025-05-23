import serial
import struct

# Serial port config (change to your actual port, e.g., 'COM3' or '/dev/ttyUSB0')
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
PACKET_SIZE = 256  # 64 floats × 4 bytes

index_map = [
    'CMU1_temp', 'Cell1_temp', 'CMU2_temp', 'Cell2_temp', 'CMU3_temp', 'Cell3_temp', 'CMU4_temp', 'Cell4_temp', 'CMU5_temp', 'Cell5_temp',
    'SOC_Ah', 'Precharge_Contactor_Driver_Status', 'Precharge_State', 'Pack_Voltage', 'Pack_Current', 'BMS_Extended_Error_Flags',
    'MC_Limit_FLags', 'MC_Error_Flags', 'Bus_Voltage', 'Bus_Current', 'Motor_Velocity', 'Vehicle_Velocity', 'PhaseC_Current', 'PhaseB_Current',
    'Motor_Temp', 'HeatSink_Temp', 'DSP_Board_Temp', 'Input_Voltage_A', 'Input_Current_A', 'Output_Voltage_A', 'Output_Current_A',
    'Mosfet_tempA', 'Controller_tempA', 'Error_Flags_A', 'Input_Voltage_B', 'Input_Current_B', 'Output_Voltage_B', 'Output_Current_B',
    'Mosfet_tempB', 'Controller_tempB', 'Error_Flags_B', 'Input_Voltage_C', 'Input_Current_C', 'Output_Voltage_C', 'Output_Current_C',
    'Mosfet_tempC', 'Controller_TempC', 'Error_Flags_C', 'Input_Voltage_D', 'Input_Current_D', 'Output_Voltage_D', 'Output_Current_D',
    'Mosfet_tempD', 'Controller_tempD', 'Error_Flags_D', 'Latitude', 'Longitude', 'Altitude', 'Speed', 'acc_X', 'acc_Y', 'acc_Z'
]

def decode_packet(packet):
    if len(packet) != PACKET_SIZE:
        print("Invalid packet size")
        return None
    
    floats = struct.unpack('<64f', packet)
    rssi, snr, *data_values = floats
    data = {name: value for name, value in zip(index_map, data_values)}
    data['RSSI'] = rssi
    data['SNR'] = snr
    return data

def read_serial():
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        while True:
            packet = ser.read(PACKET_SIZE)
            if len(packet) == PACKET_SIZE:
                decoded = decode_packet(packet)
                if decoded:
                    print(decoded)
            else:
                print("Incomplete packet")

if __name__ == "__main__":
    read_serial()
