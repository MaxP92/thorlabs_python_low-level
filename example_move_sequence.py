# -*- coding: utf-8 -*-
"""
Created on Aug 18 15:35:13 2017

@author: Maxime PINSARD
"""

## init

# copy/paste (or execute) here all the init functions

import time

## move sequence

abs_dist = 50.5 # mm
abs_dist2 = 55.5 # mm

nb_move = 60

verbose = 1
blocking = True # blocks after each move
tot_bytes2read = 20 
time_out_motion = 2 # sec

for i in range (nb_move):
    if i%2: # odd
        command_moveAbs1 = thorlabs_lowlvl_list.commandGen_moveAbsXY_meth(1, abs_dist)
    else: # even
        command_moveAbs1 = thorlabs_lowlvl_list.commandGen_moveAbsXY_meth(1, abs_dist2)

    motor_stageXY.write(command_moveAbs1)
        
    if blocking: # wait for MGMSG_MOT_MOVE_COMPLETED
        bb = b'' # binary empty
        st_time = time.time()
        while (len(bb) < tot_bytes2read): # not bool(bb):
        
            if (time_out_motion > 0 and (time.time() - st_time) >= time_out_motion): 
                print('\n read move complete timed out for motor %d !! \n' % str_motor_stop)
                print(bb)
                break
            
            bb_new = motor_stageXY.read(tot_bytes2read - len(bb)) # try to read the MOVE_COMPLETED string, during time_out seconds only (short time) !
            motor_stageXY.write(thorlabs_lowlvl_list.command_serv_alive1)
            bb = bb + bb_new
            # # print(bb)
                
            
