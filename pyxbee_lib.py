
from serial import Serial 
import asyncio
LOW_SPEED_BR = 9600
HI_SPEED_BR = 115200
DELIMITER_VALUE	= 0x7E
LOCAL_AT_COMMAND_RQST = 0x09
FRAME_ID_QUERY = 0x01
READ_TIMEOUT=5
INTER_BYTE_TIMEOUT=0.05
MAX_RESP_LEN=512
MIN_RESP_LEN=9
STATUS_OK=0x00
STATUS_OFFSET=7
FRAME_TYPE_OFFSET=3
RCV_PKT_FRAME_TYPE=0x90
SOURCE_ADDRESS_OFFSET=0x08
PAYLOAD_OFFSET=0x0F
TX_REQ=0x10
FRAME_ID_ACK=0x11
TRANSMIT_STATUS_LEN=0x0B
EXTENDED_TRANSMIT_STATUS_ID=0x8B

def xbee_compose_cmd(cmdID, cmdData):
    cmdlen = 4 + len(cmdData)
    cmd = [DELIMITER_VALUE, cmdlen >> 8, cmdlen & 0x00FF, LOCAL_AT_COMMAND_RQST, FRAME_ID_QUERY]
    cmdIDHex = [ord(i) for i in cmdID]
    cmd = cmd + cmdIDHex
    cmd = cmd + cmdData
    checksum = 0
    for i in range(3, len(cmd)):
        checksum = checksum + cmd[i]

    checksum = 0xFF - checksum & 0xFF
    cmd.append(checksum)
    return cmd

class xbee():

    def __init__(self, port, rxcallback=0):
        self.rx_packet_after_tx = []
        self.port = port
        self.rxcallback = rxcallback
    
    def xbee_uart_init(self):
        self.ser = Serial(self.port, baudrate=HI_SPEED_BR, timeout=READ_TIMEOUT, inter_byte_timeout=INTER_BYTE_TIMEOUT)
    
    def xbee_uart_deinit(self):
        self.ser.close()

    def xbee_get_power_level(self):
        power_level = 0
        cmd = xbee_compose_cmd("PL", [])
        self.ser.write(cmd)
        resp = self.ser.read(MAX_RESP_LEN)
        if MIN_RESP_LEN <= len(resp):
            if STATUS_OK == resp[STATUS_OFFSET]:
                power_level = resp[8]        

        return power_level
    
    def xbee_apply_settings(self):
        ret = False
        cmd = xbee_compose_cmd("AC", [])
        self.ser.write(cmd)
        resp = self.ser.read(MAX_RESP_LEN)
        if MIN_RESP_LEN == len(resp):
            if STATUS_OK == resp[STATUS_OFFSET]:
                ret = True     

        return ret

    def xbee_set_power_level(self, power_level):
        ret = False
        cmd = xbee_compose_cmd("PL", [power_level])
        self.ser.write(cmd)
        resp = self.ser.read(MAX_RESP_LEN)
        if MIN_RESP_LEN == len(resp):
            if STATUS_OK == resp[STATUS_OFFSET]:
                ret = True     

        return ret

    def xbee_set_pan_id(self, pan_id):
        ret = False
        cmd_data = [(pan_id >> 8) & 0x00FF, pan_id & 0x00FF]
        cmd = xbee_compose_cmd("ID", cmd_data)
        self.ser.write(cmd)
        resp = self.ser.read(MAX_RESP_LEN)
        if MIN_RESP_LEN == len(resp):
            if STATUS_OK == resp[STATUS_OFFSET]:
                ret = True     

        return ret

    def xbee_receive_packet(self):
        source_address = []  
        payload = []

        if 0 < len(self.rx_packet_after_tx):
            packet = self.rx_packet_after_tx
            self.rx_packet_after_tx = []
        else:
            packet = self.ser.read(MAX_RESP_LEN)

        if MIN_RESP_LEN < len(packet):
            frame_len = (packet[1] << 8) + packet[2]
            if frame_len < len(packet): 
                if RCV_PKT_FRAME_TYPE == packet[FRAME_TYPE_OFFSET]:   
                    source_address = list(packet[SOURCE_ADDRESS_OFFSET : SOURCE_ADDRESS_OFFSET + 4])
                    payload = list(packet[PAYLOAD_OFFSET : PAYLOAD_OFFSET + frame_len - 12])
        return [source_address, payload]

    def xbee_tx_packet(self, dest_address, payload):
        ret = False
        tx_frame = []
        addr = [0x00, 0x13, 0xA2, 0x00]
        length = 14 + len(payload)
        tx_frame.append(DELIMITER_VALUE)
        tx_frame.append((length & 0x0FF00) >> 8)
        tx_frame.append(length & 0x000FF)
        tx_frame.append(TX_REQ)
        tx_frame.append(FRAME_ID_ACK)
        addr = addr + dest_address
        tx_frame = tx_frame + addr
        tx_frame.append(0xFF)
        tx_frame.append(0xFE)
        tx_frame.append(0x00)
        tx_frame.append(0xC0)
        tx_frame = tx_frame + payload
        checksum = 0
        for i in range(3, len(tx_frame)):
            checksum = checksum + tx_frame[i]

        checksum = 0xFF - checksum & 0xFF
        tx_frame.append(checksum)
        self.ser.write(tx_frame)

        resp = self.ser.read(MAX_RESP_LEN)
        if TRANSMIT_STATUS_LEN <= len(resp):
            if (DELIMITER_VALUE == resp[0]) and (EXTENDED_TRANSMIT_STATUS_ID == resp[3]) and (0x00 == resp[8]): 
                ret = True
        
        ### Check if another RX message came
        if TRANSMIT_STATUS_LEN <= len(resp):
            self.rx_packet_after_tx = resp[TRANSMIT_STATUS_LEN : len(resp) - 1]
        else:
            self.rx_packet_after_tx = []
        return ret

    