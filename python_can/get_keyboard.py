import keyboard
import time

count = 0
SP = 0

# def change_speed(e, SP):
#     print(e)
#     if e.name == 'up':
#         print ('UP')
#         #dSP = 'uup'
#         print(SP)
#     if e.name == 'down':
#         print('D')
#         dSP = 'dd'
#     return dSP

# #while True:
# #    
# #    keyboard.on_release(change_speed(0))
# #    print("Working..." + str(count))
# #    time.sleep(1)
# SP=1
# dSP = keyboard.on_release(change_speed(SP))
# print(dSP)
# time.sleep(7)
SP_speed = 0
while SP_speed != 'q':

    SP_speed = input('Set speed: ')
    
    print (SP_speed)
    #send command