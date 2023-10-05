import canopen
import time
#import keyboard

from canopen.profiles.p402 import BaseNode402



node = BaseNode402(2, '/home/pi/catkin_ws/src/maxon_epos4_ros1/python_can/maxon_motor_EPOS4_0110h_6551h_0000h_0000h_NODE2.eds')
node2 = BaseNode402(1, '/home/pi/catkin_ws/src/maxon_epos4_ros1/python_can/maxon_motor_EPOS4_0110h_6551h_0000h_0000h_NODE1.eds')

network = canopen.Network()
network.add_node(node)
network.add_node(node2)

network.connect(bustype='socketcan', channel='can0', bitrate=250000)

node.nmt.state = 'RESET COMMUNICATION'  # 000h 82 01
node2.nmt.state = 'RESET COMMUNICATION'  # 000h 82 01
print(node.nmt.state + " | " + node2.nmt.state)
time.sleep(5)

node.tpdo.read()
node.rpdo.read()
node2.tpdo.read()
node2.rpdo.read()

node.load_configuration()
node.setup_402_state_machine()

node2.load_configuration()
node2.setup_402_state_machine()

print(node.state  + " | " + node2.state)
node.nmt.state = 'OPERATIONAL'
node2.nmt.state = 'OPERATIONAL'

time.sleep(2)
network.sync.start(1)       #SYNC
time.sleep(1)
print(node.state)

node.rpdo[1]['Controlword'].raw = 0x06
node.rpdo[1].transmit()
node2.rpdo[1]['Controlword'].raw = 0x06
node2.rpdo[1].transmit()
time.sleep(2)
print('Node 1: ' + node.state + ' / ' + node.nmt.state)
print('Node 2: ' + node2.state + ' / ' + node2.nmt.state)

#node.rpdo[1]['Controlword'].raw = 0x07
#node.rpdo[1].transmit()
SP_speed = 0x100
count = 0

node.rpdo[1]['Controlword'].raw = 0x0F
node2.rpdo[1]['Controlword'].raw = 0x0F
SP_speed = 0
while SP_speed != 'q':
     SP_speed = input('Set speed: ')
     if SP_speed == 'q':
          break
     else:
          #node.rpdo[1]['Target velocity'].raw = SP_speed #set speed
          node.rpdo[1]['Target velocity'].phys = int(SP_speed)
          node.rpdo[1].transmit()
          node2.rpdo[1]['Target velocity'].phys = int(SP_speed)
          node2.rpdo[1].transmit()
     
     time.sleep(1)
     
     speed = node.tpdo[3]['Velocity actual values.Velocity actual value averaged'].phys
     speed2 = node2.tpdo[3]['Velocity actual values.Velocity actual value averaged'].phys
     
     print('Speed 1: ' + str(speed) + ' | SP: ' + str(SP_speed))
     print('Speed 2: ' + str(speed2) + ' | SP: ' + str(SP_speed))


node.rpdo[1]['Target velocity'].raw = 0
node.rpdo[1].transmit()
node2.rpdo[1]['Target velocity'].raw = 0
node2.rpdo[1].transmit()
time.sleep(1)
######

#node.state = 'SWITCHED_ON'

#node.sdo[0x6040].raw = 0x06
##node.sdo[0x6040].raw = 0x07
#node.sdo[0x6040].raw = 0x0F   #ON

#node.nmt.state = 'OPERATIONAL'
#print(node.nmt.state)

#node.sdo[0x6040].raw = 0x80   #reset fault

#node.rpdo[1]['Target velocity'].raw = 0x0500 #set speed
#node.rpdo[1].transmit()

#Print Actual Speed
#print (node.tpdo[2]['Velocity actual value'].phys)



#for node_id in network:
#     print(network[node_id])


#for obj in node.object_dictionary.values():
#    print('0x%X: %s' % (obj.index, obj.name))
#    if isinstance(obj, canopen.objectdictionary.Record):
#        for subobj in obj.values():
#            print('  %d: %s' % (subobj.subindex, subobj.name))



network.disconnect()


