## Shelly Pro 3em to  Fronius Modbus
# Based on:
# https://github.com/ciedema/froniusmodbusimulation

# Using a Shelly Pro 3em configured in the triphase profile
#
# Pass in the Shelly IP in the environment SHELLY_PRO3EM

# L1 = C   (required to power Shelly Pro 3EM)
# L2 = B
# L3 = A


from pprint import pprint
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusSocketFramer, ModbusAsciiFramer
from pymodbus.server import StartTcpServer
from pymodbus.server import StartAsyncTcpServer
import threading
import struct
import time
import json
import getopt
import sys
import socket
import signal
import os
import urllib.request
import asyncio

###############################################################
# Timer Class
###############################################################
class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = threading.Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

meters = [ os.environ.get('SHELLY_PRO3EM', '127.0.0.1') ]

context = []
servers =[]
modbus_port=502
corrfactor = 1000
i_corrfactor = int(corrfactor)

rtime = 0


def update_meters():
    #global meterdict
    global ep_int1
    global ep_int2
    global exp_int1
    global exp_int2
    global ti_int1
    global ti_int2

    for ip in meters:
        with urllib.request.urlopen("http://"+ip+"/rpc/Shelly.GetStatus") as url:
            meterdata=json.load(url)

        # Current = A
        float_current_L1 = float(meterdata["em:0"]["c_current"])
        float_current_L2 = float(meterdata["em:0"]["b_current"])
        float_current_L3 = float(meterdata["em:0"]["a_current"])
        # Voltage = V
        float_voltage_L1 = float(meterdata["em:0"]["c_voltage"])
        float_voltage_L2 = float(meterdata["em:0"]["b_voltage"])
        float_voltage_L3 = float(meterdata["em:0"]["a_voltage"])
        # Actual Power = W
        float_act_power_L1 = float(meterdata["em:0"]["c_act_power"])
        float_act_power_L2 = float(meterdata["em:0"]["b_act_power"])
        float_act_power_L3 = float(meterdata["em:0"]["a_act_power"])
        # Apparent Power = VA
        float_apt_power_L1 = float(meterdata["em:0"]["c_aprt_power"])
        float_apt_power_L2 = float(meterdata["em:0"]["b_aprt_power"])
        float_apt_power_L3 = float(meterdata["em:0"]["a_aprt_power"])
        # Power Factor
        float_pf_L1 = float(meterdata["em:0"]["c_pf"])
        float_pf_L2 = float(meterdata["em:0"]["b_pf"])
        float_pf_L3 = float(meterdata["em:0"]["a_pf"])
        # Totals, Actual = W, Appparent = VA
        float_total_current = float(meterdata["em:0"]["total_current"])
        float_total_act_power = float(meterdata["em:0"]["total_act_power"])
        float_total_apt_power = float(meterdata["em:0"]["total_aprt_power"])
        float_voltage = float(meterdata["em:0"]["c_voltage"])
        float_freq = float(meterdata["em:0"]["c_freq"])

        # EMData - Wh values (need to multiply by correction factor??)
        # (need to multiply by correction factor??)
        # (unsure, so override to 1 for now
        i_corrfactor = 1
        float_total_actual_energy_L1 = float(meterdata["emdata:0"]["c_total_act_energy"]) * i_corrfactor
        float_total_returned_energy_L1 = float(meterdata["emdata:0"]["c_total_act_ret_energy"]) * i_corrfactor
        float_total_actual_energy_L2 = float(meterdata["emdata:0"]["b_total_act_energy"]) * i_corrfactor
        float_total_returned_energy_L2 = float(meterdata["emdata:0"]["b_total_act_ret_energy"]) * i_corrfactor
        float_total_actual_energy_L3 = float(meterdata["emdata:0"]["a_total_act_energy"]) * i_corrfactor
        float_total_returned_energy_L3 = float(meterdata["emdata:0"]["a_total_act_ret_energy"]) * i_corrfactor
        float_total_actual_energy = float(meterdata["emdata:0"]["total_act"]) * i_corrfactor
        float_total_returned_energy = float(meterdata["emdata:0"]["total_act_ret"]) * i_corrfactor

        # Converting values of smart meter out of payload into Modbus register
        # Pack into binary data as float, then pull out as integers... ?
        # Then convert to hex
        # Then break apart HEX into two register parts
        # Then convert back to integer because pymodbus converts to hex
        def make_mb_parts( value ):
            # print( "Incoming value: " + str( value ) )
            if value == 0:
                int1 = 0
                int2 = 0
            else:
                vhex = hex(struct.unpack('<I', struct.pack('<f', value))[0])
                # print( "Vhex: " + str(vhex) )
                vhex_p1 = str(vhex)[2:6]
                vhex_p2 = str(vhex)[6:10]
                # print( "Vhex parts: " + str(vhex_p1) + "," + str(vhex_p2))
                int1 = int(vhex_p1, 16)
                int2 = int(vhex_p2, 16)
            # print( "Returning: " + str(int1) + "," + str(int2) )
            return int1, int2

        cur_L1_int1, cur_L1_int2 = make_mb_parts( float_current_L1 )
        cur_L2_int1, cur_L2_int2 = make_mb_parts( float_current_L2 )
        cur_L3_int1, cur_L3_int2 = make_mb_parts( float_current_L3 )
        vlt_L1_int1, vlt_L1_int2 = make_mb_parts( float_voltage_L1 )
        vlt_L2_int1, vlt_L2_int2 = make_mb_parts( float_voltage_L2 )
        vlt_L3_int1, vlt_L3_int2 = make_mb_parts( float_voltage_L3 )
        act_L1_int1, act_L1_int2 = make_mb_parts( float_act_power_L1 )
        act_L2_int1, act_L2_int2 = make_mb_parts( float_act_power_L2 )
        act_L3_int1, act_L3_int2 = make_mb_parts( float_act_power_L3 )
        apt_L1_int1, apt_L1_int2 = make_mb_parts( float_apt_power_L1 )
        apt_L2_int1, apt_L2_int2 = make_mb_parts( float_apt_power_L2 )
        apt_L3_int1, apt_L3_int2 = make_mb_parts( float_apt_power_L3 )
        pf_L1_int1, pf_L1_int2 = make_mb_parts( float_pf_L1)
        pf_L2_int1, pf_L2_int2 = make_mb_parts( float_pf_L2)
        pf_L3_int1, pf_L3_int2 = make_mb_parts( float_pf_L3)
        volt_int1, volt_int2 = make_mb_parts( ( float_voltage_L1 + float_voltage_L2 + float_voltage_L3 ) / 3 )
        freq_int1, freq_int2 = make_mb_parts( float_freq)
        curr_int1, curr_int2 = make_mb_parts( float_total_current)

        ti_int1, ti_int2 = make_mb_parts( float_total_actual_energy ) 
        ti_L1_int1, ti_L1_int2 = make_mb_parts( float_total_actual_energy_L1 ) 
        ti_L2_int1, ti_L2_int2 = make_mb_parts( float_total_returned_energy_L2 ) 
        ti_L3_int1, ti_L3_int2 = make_mb_parts( float_total_returned_energy_L3 ) 
        te_int1, te_int2 = make_mb_parts( float_total_returned_energy ) 
        te_L1_int1, te_L1_int2 = make_mb_parts( float_total_returned_energy_L1 ) 
        te_L2_int1, te_L2_int2 = make_mb_parts( float_total_returned_energy_L2 ) 
        te_L3_int1, te_L3_int2 = make_mb_parts( float_total_returned_energy_L3 ) 
        tact_int1, tact_int2 = make_mb_parts( float_total_act_power )
        tapt_int1, tapt_int2 = make_mb_parts( float_total_apt_power )

        register = 3
        slave_id = 0x01
        address = 0x9C87
        values = [curr_int1, curr_int2,               #Ampere - AC Total Current Value [A]
                  cur_L1_int1, cur_L1_int2,               #Ampere - AC Current Value L1 [A]
                  cur_L2_int1, cur_L2_int2,               #Ampere - AC Current Value L2 [A]
                  cur_L3_int1, cur_L3_int2,               #Ampere - AC Current Value L3 [A]
                  volt_int1, volt_int2,               #Voltage - Average Phase to Neutral [V]
                  vlt_L1_int1, vlt_L1_int2,               #Voltage - Phase L1 to Neutral [V]
                  vlt_L2_int1, vlt_L2_int2,               #Voltage - Phase L2 to Neutral [V]
                  vlt_L3_int1, vlt_L3_int2,               #Voltage - Phase L3 to Neutral [V]
                  0, 0,               #Voltage - Average Phase to Phase [V]
                  0, 0,               #Voltage - Phase L1 to L2 [V]
                  0, 0,               #Voltage - Phase L2 to L3 [V]
                  0, 0,               #Voltage - Phase L1 to L3 [V]
                  freq_int1, freq_int2,               #AC Frequency [Hz]
                  tact_int1, tact_int2,         #AC Power value (Total) [W] ==> Second hex word not needed
                  act_L1_int1, act_L1_int2,               #AC Power Value L1 [W]
                  act_L2_int1, act_L2_int2,               #AC Power Value L2 [W]
                  act_L3_int1, act_L3_int2,               #AC Power Value L3 [W]
                  tapt_int1, tapt_int2,               #AC Apparent Power [VA]
                  apt_L1_int1, apt_L1_int2,               #AC Apparent Power L1 [VA]
                  apt_L2_int1, apt_L2_int2,               #AC Apparent Power L2 [VA]
                  apt_L3_int1, apt_L3_int2,               #AC Apparent Power L3 [VA]
                  0, 0,               #AC Reactive Power [VAr]
                  0, 0,               #AC Reactive Power L1 [VAr]
                  0, 0,               #AC Reactive Power L2 [VAr]
                  0, 0,               #AC Reactive Power L3 [VAr]
                  0, 0,               #AC power factor total [cosphi]
                  pf_L1_int1, pf_L1_int2,               #AC power factor L1 [cosphi]
                  pf_L2_int1, pf_L2_int2,               #AC power factor L2 [cosphi]
                  pf_L3_int1, pf_L3_int2,               #AC power factor L3 [cosphi]
                  te_int1, te_int2, #Total Watt Hours Exportet [Wh]
                  te_L1_int1, te_L1_int2,               #Watt Hours Exported L1 [Wh]
                  te_L2_int1, te_L2_int2,               #Watt Hours Exported L2 [Wh]
                  te_L3_int1, te_L3_int2,               #Watt Hours Exported L3 [Wh]
                  ti_int1, ti_int2,   #Total Watt Hours Imported [Wh]
                  ti_L1_int1, ti_L1_int2,               #Watt Hours Imported L1 [Wh]
                  ti_L2_int1, ti_L2_int2,               #Watt Hours Imported L2 [Wh]
                  ti_L3_int1, ti_L3_int2,               #Watt Hours Imported L3 [Wh]
                  0, 0,               #Total VA hours Exported [VA]
                  0, 0,               #VA hours Exported L1 [VA]
                  0, 0,               #VA hours Exported L2 [VA]
                  0, 0,               #VA hours Exported L3 [VA]
                  0, 0,               #Total VAr hours imported [VAr]
                  0, 0,               #VA hours imported L1 [VAr]
                  0, 0,               #VA hours imported L2 [VAr]
                  0, 0                #VA hours imported L3 [VAr]
                  ]

        context[0].setValues(register, address, values)
        #pprint( values )


def start_meter(emeter, address):

    print ( "Address:" )
    print(address)
    global context
    StartTcpServer(
        context=context,
        address=address,
        framer=ModbusSocketFramer,
    )

def setup_meter ():
    global context
    global modbus_port
    global servers

    datablock = ModbusSparseDataBlock({

        40001:  [21365, 28243],
        40003:  [1],
        40004:  [65],
        40005:  [70,114,111,110,105,117,115,0,0,0,0,0,0,0,0,0,         #Manufacturer "Fronius
                83,109,97,114,116,32,77,101,116,101,114,32,54,51,65,0, #Device Model "Smart Meter
                0,0,0,0,0,0,0,0,                                       #Options N/A
                0,0,0,0,0,0,0,0,                                       #Software Version  N/A
                48,48,48,48,48,48,52,50,0,0,0,0,0,0,0,0,               #Serial Number: 00000042
                240],                                                  #Modbus TCP Address:
        40070: [213],
        40071: [124],
        40072: [0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,0,0,0,
                0,0,0,0],

        40196: [65535, 0],
    })
    slaveStore = ModbusSlaveContext(
        di=datablock,
        co=datablock,
        hr=datablock,
        ir=datablock,
    )
    context = ModbusServerContext(slaves=slaveStore, single=True)

    ###############################################################
    # Run Update Register every 5 Seconds
    ###############################################################
    time = 5  # 5 seconds delay
    rt = RepeatedTimer(time, update_meters, )

    #ip_address="192.168.20.21"+str(emeter)
    #ip_address="192.168.42."+str(241+emeter)
    #ip_address="192.168.99.164"
    ip_address="0.0.0.0" # Docker container
    print("### start server, listening on "+ip_address+":"+str(modbus_port))
    address = (ip_address, modbus_port)
    #print( "Address:" )
    #print(address)
    x=threading.Thread(target=start_meter,args=(0,address))
    x.start()
#    servers.append(await StartAsyncTcpServer(
#        context=contexts[emeter],
#        address=address,
#        framer=ModbusSocketFramer,
#        allow_reuse_addyress=True,
#    ))




if __name__ == "__main__":
    setup_meter()
