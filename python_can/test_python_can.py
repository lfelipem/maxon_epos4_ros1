import canopen
import time


network = canopen.Network()

network.connect(bustype='socketcan', channel='can0', bitrate=500000)

#add node and DCF File
node = network.add_node(2, 'maxon_motor_EPOS4_0110h_6551h_0000h_0000h.eds')
network.add_node(node)

node.nmt.state = 'RESET COMMUNICATION'  # 000h 82 01
print(node.nmt.state)
time.sleep(5)

node.nmt.state = 'OPERATIONAL'
print(node.nmt.state)


for node_id in network:
     print(network[node_id])

#node.tpdo.read()
#node.rpdo.read()

#network.sync.start(0.05)

#for obj in IO_module.object_dictionary.values():
#    print('0x%X: %s' % (obj.index, obj.name))
#    if isinstance(obj, canopen.objectdictionary.Record):
#        for subobj in obj.values():
#            print('  %d: %s' % (subobj.subindex, subobj.name))

#IO_module.load_configuration()
network.disconnect()


