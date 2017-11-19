# -*- coding: utf-8 -*-
"""
Created on Aug 18 15:35:13 2017

@author: Maxime PINSARD
"""

## init

# copy/paste (or execute) here all the init functions

## move sequence

abs_dist = 50.5 # mm
abs_dist2 = 55.5 # mm

nb_move = 60

verbose = 1
blocking = True # blocks after each move

for i in range (nb_move):
    if i%2: # odd
        command_moveAbs1 = thorlabs_lowlvl_list.commandGen_moveAbsXY_meth(1, abs_dist)
    else: # even
        command_moveAbs1 = thorlabs_lowlvl_list.commandGen_moveAbsXY_meth(1, abs_dist2)

    motor_stageXY.write(command_moveAbs1)
    if blocking: # wait for MGMSG_MOT_MOVE_COMPLETED
        bb = motor_stageXY.read(20)
        if verbose:
            pos , stat = thorlabs_lowlvl_list.msg_decoder_movecompleted_XY_meth(bb) 
            print('Move pos is %s with state %s', pos , stat)
            
        motor_stageXY.write(thorlabs_lowlvl_list.command_serv_alive1) # say that server is alive
