# -*- coding: utf-8 -*-
"""
Created on Aug 15 15:35:13 2017

@author: Maxime PINSARD
"""

'''
low-level commands, originally for a BBD102 controller
Adapted for thorlabs motor controllers
For other devices (K-cubes, piezo..), see APT Com. Protocol doc to adapt your functions

tips : ch1 = x21 = ! in utf-8 hx
tips : ch2 = x22 = " in utf-8 hx
'''

# import struct
import array

sourceID = b'\x01' # your PC

channel_ident_short = b'\x01' # is always 01, the bay design the real channel

channel_ident_long = b'\x01\x00' # is always 01, the bay design the real channel

destinationID_CH1 = b'\x21' # channel 1 on bay 1 = CH1
destinationID_CH2 = b'\x22' # channel 1 on bay 2 = CH2
destinationID_gen =  b'\x11'

destinationID_CH1_var = b'\xA1' # channel 1 on bay 1 = CH1
destinationID_CH2_var = b'\xA2' # channel 1 on bay 2 = CH2

destinationID_USB = b'\x50'

MLS203_EncCnt_per_mm = 20000

MLS203_scfactor_vel = 134217.73
MLS203_scfactor_acc = 13.744

jerk_default = b'\xe1\x12\x0e\x00' # 10,000 mm/sec3 (i.e. 922337)

not_used_key = b'\x00\x00\x00\x00'

# 'I' for uint, 'L' for ulong, 'f' for float, 
# long for thorlabs is Signed 32 bit integer (4 bytes) ie 'l'
# dword for thorlabs is  Unsigned 32 bit integer (4 bytes)
# short for thorlabs is  Signed 16 bit integer (2 bytes)
# word for thorlabs is Unsigned 16 bit integer (2 bytes)
# Thorlabs APT Controllers Host-Controller Communications Protocol, Issue 20

## General function

def write_and_read_meth(channel, motor_stageXY, cmd_write, nb_bt_read):
    
    motor_stageXY.write(cmd_write)
    bb = ''
    while not bool(bb):
        bb = motor_stageXY.read(nb_bt_read) 
    
    return bb

## channels MGMSG_MOD_SET_CHANENABLESTATE

msg_ID_channelEnable = b'\x10\x02'

command_En0 = msg_ID_channelEnable + channel_ident_short + b'\x01' + destinationID_gen + sourceID
command_EnCH1 = msg_ID_channelEnable + channel_ident_short + b'\x01' + destinationID_CH1 + sourceID
command_EnCH2 = msg_ID_channelEnable + channel_ident_short + b'\x01' + destinationID_CH2 + sourceID
# command_EnCH2 = msg_ID_channelEnable + b'\x02' + b'\x01' + destinationID_CH2 + sourceID

## Req channel state MGMSG_MOD_REQ_CHANENABLESTATE

msg_ID_channelReq = b'\x11\x02'

command_req_ch0 = msg_ID_channelReq + channel_ident_long + destinationID_gen + sourceID # does not work !
command_req_ch1 = msg_ID_channelReq + channel_ident_long + destinationID_CH1 + sourceID
command_req_ch2 = msg_ID_channelReq + channel_ident_long + destinationID_CH2 + sourceID

def get_chstate_bycommand_meth(channel, motor_stageXY):
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2
    elif channel == 0: # gen
        destinationID_CH = destinationID_gen    # does not work ! 
        
    command_req_chstate = msg_ID_channelReq + channel_ident_long + destinationID_CH + sourceID
    # print(command_req_chstate)
    motor_stageXY.write(command_req_chstate)
    bb = ''
    while not bool(bb):
        bb = motor_stageXY.read(6) 
        
    state = bb[3]
     
    return state

## homing 

msg_ID_home = b'\x43\x04' 

command_home1 = msg_ID_home + channel_ident_long + destinationID_CH1 + sourceID
command_home2 = msg_ID_home + channel_ident_long + destinationID_CH2 + sourceID

## has been homed : it's a READ !!

msg_ID_hasBeenHomed = b'\x44\x04' 

command_hasBeenHomed1 = msg_ID_hasBeenHomed + channel_ident_long + destinationID_CH1 + sourceID
command_hasBeenHomed2 = msg_ID_hasBeenHomed + channel_ident_long + destinationID_CH2 + sourceID


# unfortunately is emited only after homing

## get velocity parameters 

msg_ID_req_velparam = b'\x14\x04'

def get_velparam_bycommand_meth(channel, motor_stageXY):
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2
        
    command_req_velparam = msg_ID_req_velparam + channel_ident_long + destinationID_CH + sourceID
    
    motor_stageXY.write(command_req_velparam)
    bb = ''
    while not bool(bb):
        bb = motor_stageXY.read(20) 
        
    arr = array.array('l', bb) # [8:19]
    min_vel = arr[2]/MLS203_scfactor_vel
    max_acc = arr[3]/MLS203_scfactor_acc
    max_vel = arr[4]/MLS203_scfactor_vel
    # print(min_vel, max_acc, max_vel)
     
    return min_vel, max_acc, max_vel

## set velocity parameters 

msg_ID_setvelparam = b'\x13\x04\x0E\x00' 

min_vel_ini = b'\x00\x00\x00\x00' # always 0, does nothing otherwise

def commandGen_setvelparamXY_meth(channel, max_vel, acc_max):
    
    yy = array.array('l', [round(max_vel*MLS203_scfactor_vel)])
    # y.byteswap()
    vel_max_hex = yy.tostring()
    # abs_dist_hex =  = y.decode('utf-8')
    
    y = array.array('l', [round(acc_max*MLS203_scfactor_acc)])
    # y.byteswap()
    acc_max_hex = y.tostring()
    # abs_dist_hex =  = y.decode('utf-8')
    
    if channel == 1: # x
        destinationID_CH_var = destinationID_CH1_var
        
    elif channel == 2: # y
        destinationID_CH_var = destinationID_CH2_var
    
    command_setvelparam = msg_ID_setvelparam + destinationID_CH_var + sourceID + channel_ident_long + min_vel_ini
    
    command_setvelparam = command_setvelparam + acc_max_hex + vel_max_hex
    
    return command_setvelparam

## req HW trigger 

msg_ID_req_trigger = b'\x01\x05'  # for the request, the module responds with a get

command_req_trigger1 = msg_ID_req_trigger + channel_ident_long + destinationID_CH1 + sourceID
command_req_trigger2 = msg_ID_req_trigger + channel_ident_long + destinationID_CH2 + sourceID

def get_trig_bycommand_meth(channel, motor_stageXY):
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2
        
    command_req_trigger = msg_ID_req_trigger + channel_ident_long + destinationID_CH + sourceID
    
    motor_stageXY.write(command_req_trigger)
    bb = ''
    while not bool(bb):
        bb = motor_stageXY.read(6) 
        
    state = bin(bb[3])
     
    return state

## set HW trigger 

msg_ID_set_trigger = b'\x00\x05'  # for the request, the module responds with a get

# trigger_mode_str can be (see APT com. protocol) :
# - TRIGIN HIGH – IN REL MOVE – none- none - TRIGOUT HIGH – none – MOTION COMP- none
# 1-1-0-0-1-0-1-0 in reverse way so 01010011 , hex(int('01010011',2)) = '0x53'
# 
# - none – none – none- none - TRIGOUT HIGH – none – none - Max Vel. 
# 0-0-0-0-1-0-0-1 in reverse way so 10010000 , hex(int('10010000',2)) = '0x90'
#
# - others

key_trigout_maxvelreached = '10010000'
verif_trigout_maxvelreached = '0b10010000'
lbl_trig_maxvelreached = 'Max Vel. reached'
key_trigout_inmotion = '00110000'
verif_trigout_inmotion = '0b110000'
lbl_trig_inmotion = 'In Motion'

def set_trig_meth(channel, motor_stageXY, trigger_mode_str):
    
    y = array.array('B', [int(trigger_mode_str, 2)]) #  B = unsigned char
    mode = y.tostring()
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2
    
    command_set_trigger = msg_ID_set_trigger + channel_ident_short + mode + destinationID_CH + sourceID
    
    motor_stageXY.write(command_set_trigger)

## set profile BBD202 : MGMSG_MOT_SET_BOWINDEX

# you can set the move profile to trapezoidal (brutal, but linear), or to S-curve (smoother, non-linear)

msg_ID_set_profile = b'\xF4\x04\x04\x00'  # for the request, the module responds with a get

profile_type = b'\x00\x00' # first 0 for trapezidal, 1-18 for S-curve, 2nd is always 0

command_set_profile_trapez1 = msg_ID_set_profile + destinationID_CH1_var + sourceID + channel_ident_long + profile_type
command_set_profile_trapez2 = msg_ID_set_profile + destinationID_CH2_var + sourceID + channel_ident_long + profile_type


## req profile BBD202 : MGMSG_MOT_REQ_BOWINDEX

# WARNING : it does not seem to work

msg_ID_req_profile = b'\xF5\x04'  # for the request, the module responds with a get

command_req_profile1 = msg_ID_req_profile + channel_ident_long + destinationID_CH1 + sourceID
command_req_profile2 = msg_ID_req_profile + channel_ident_long + destinationID_CH2 + sourceID

## set profile BBD102 : MGMSG_MOT_SET_PMDPROFILEMODEPARAMS

# you can set the move profile to trapezoidal (brutal, but linear), or to S-curve (smoother, non-linear)

msg_ID_set_profile = b'\xE3\x04\x0C\x00'  # for the request, the module responds with a get

profile_type = b'\x00\x00' # first 0 for trapezidal, 1-18 for S-curve, 2nd is always 0

command_set_profile_old_trapez1 = msg_ID_set_profile + destinationID_CH1_var + sourceID + channel_ident_long + profile_type + jerk_default + not_used_key
command_set_profile_old_trapez2 = msg_ID_set_profile + destinationID_CH2_var + sourceID + channel_ident_long + profile_type + jerk_default + not_used_key

profile_type_scurve = b'\x02\x00' # first 0 for trapezidal, 1-18 for S-curve, 2nd is always 0

command_set_profile_old_scurve1 = msg_ID_set_profile + destinationID_CH1_var + sourceID + channel_ident_long + profile_type_scurve + jerk_default + not_used_key
command_set_profile_old_scurve2 = msg_ID_set_profile + destinationID_CH2_var + sourceID + channel_ident_long + profile_type_scurve + jerk_default + not_used_key

## req profile BBD102 : MGMSG_MOT_REQ_PMDPROFILEMODEPARAMS


msg_ID_req_profile = b'\xE4\x04'  # for the request, the module responds with a get

key_prof_trapez1 = command_set_profile_old_trapez1
key_prof_trapez2 = command_set_profile_old_trapez2 
verif_prof_trapez = 0 
lbl_prof_trapez = 'trapezoidal'

key_prof_scurve1 = command_set_profile_old_scurve1
key_prof_scurve2 = command_set_profile_old_scurve2
verif_prof_scurve  = 2
lbl_prof_scurve = 'S-curve'

command_req_profile_old_1 = msg_ID_req_profile + channel_ident_long + destinationID_CH1 + sourceID
command_req_profile_old_2 = msg_ID_req_profile + channel_ident_long + destinationID_CH2 + sourceID

def get_profile_bycommand_meth(channel, motor_stageXY):
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2
        
    command_req_profile = msg_ID_req_profile + channel_ident_long + destinationID_CH + sourceID
    
    motor_stageXY.write(command_req_profile)
    bb = ''
    while not bool(bb):
        bb = motor_stageXY.read(18) 
    prof = bb[8] # bin(
     
    return prof


## req position  : MGMSG_MOT_REQ_POSCOUNTER

msg_ID_req_position = b'\x11\x04'  # for the request, the module responds with a get

# command_req_position2 = msg_ID_req_position + channel_ident_long + destinationID_CH2 + sourceID

# response is '\xF6\x04\04\00' + destinationID_CH2, sourceID + Chan Ident, Bow Index

def get_posXY_bycommand_meth(channel, motor_stageXY):
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2
        
    command_req_position = msg_ID_req_position + channel_ident_long + destinationID_CH + sourceID
    
    motor_stageXY.write(command_req_position)
    bb = ''
    while not bool(bb):
        bb = motor_stageXY.read(12) 
    arr = array.array('l', bb)
    pos = arr[2]/MLS203_EncCnt_per_mm
     
    return pos

## MGMSG_MOT_MOVE_COMPLETED (move abs)

# with MGMSG_MOT_GET_STATUSUPDATE
def msg_decoder_movecompleted_XY_meth(msg):
    # msg in bytes string
    
    msg = msg[8:len(msg)] # 8 first bytes are always the same
    
    msg_pos = msg[0:4]
    arr = array.array('l', msg_pos)
    pos = arr[0]/MLS203_EncCnt_per_mm
    msg_enc = msg[4:8]
    arr = array.array('l', msg_enc)
    enc = arr[0]
    msg_stat = msg[8:len(msg)] # 4 bytes
    arr = array.array('L', msg_stat) # unsigned
    stat = hex(arr[0])
    
    return pos, stat
    
## move to abs : MGMSG_MOT_MOVE_ABSOLUTE short version

#MGMSG_ MOT_SET_MOVEABSPARAMS, and after the short version call many time (see doc)

msg_ID_moveAbsparam = b'\x50\x04\x06\x00'  # for the request, the module responds with a get

# command_moveAbs1 = '%s%s%s%s%s' % (msg_ID_moveAbs, destinationID_CH1, sourceID, channel_ident) #, abs_dist)
# command_moveAbs2 = '%s%s%s%s%s' % (msg_ID_moveAbs, destinationID_CH2, sourceID, channel_ident) #, abs_dist)

# command_moveAbs1 = command_moveAbs1.encode('utf-8')
# command_moveAbs2 = command_moveAbs2.encode('utf-8')

def commandGen_moveAbsparamXY_meth(channel, abs_dist):
    # move absolute command generator
    
    # abs_dist_hex = struct.pack('>I', round(abs_dist*MLS203_EncCnt_per_mm)).decode('utf-8')  # abs_dist in mm
    
    y = array.array('l', [round(abs_dist*MLS203_EncCnt_per_mm)])
    # # y.byteswap()
    abs_dist_hex = y.tostring()
    # abs_dist_hex =  = y.decode('utf-8')
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1
        
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2
        
    command_moveAbsparam = msg_ID_moveAbsparam + destinationID_CH + sourceID + channel_ident_long + abs_dist_hex 
    
    return command_moveAbsparam

    
msg_ID_moveAbsshort = b'\x53\x04'  # for the request, the module responds with a get

def commandGen_moveAbsshort_meth(channel):
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1
        
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2

    command_moveAbsshort = msg_ID_moveAbsshort + channel_ident_long + destinationID_CH + sourceID
    
    return command_moveAbsshort

## move to abs : MGMSG_MOT_MOVE_ABSOLUTE long version

# a repetitive move can be set to MGMSG_ MOT_SET_MOVEABSPARAMS, and after the short version call many time (see doc)

msg_ID_moveAbs = b'\x53\x04\x06\x00'  

# command_moveAbs1 = '%s%s%s%s%s' % (msg_ID_moveAbs, destinationID_CH1, sourceID, channel_ident) #, abs_dist)
# command_moveAbs2 = '%s%s%s%s%s' % (msg_ID_moveAbs, destinationID_CH2, sourceID, channel_ident) #, abs_dist)

# command_moveAbs1 = command_moveAbs1.encode('utf-8')
# command_moveAbs2 = command_moveAbs2.encode('utf-8')

def commandGen_moveAbsXY_meth(channel, abs_dist):
    # move absolute command generator
    
    # abs_dist_hex = struct.pack('>I', round(abs_dist*MLS203_EncCnt_per_mm)).decode('utf-8')  # abs_dist in mm
    
    y = array.array('l', [round(abs_dist*MLS203_EncCnt_per_mm)])
    # # y.byteswap()
    abs_dist_hex = y.tostring()
    # abs_dist_hex =  = y.decode('utf-8')
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1_var
        
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2_var
        
    command_moveAbs = msg_ID_moveAbs + destinationID_CH + sourceID + channel_ident_long + abs_dist_hex 
    
    return command_moveAbs

## move to rel : MGMSG_MOT_MOVE_RELATIVE long version

# see doc, you also have a short or a long version

msg_ID_moveRel = b'\x48\x04\x06\x00'  # for the request, the module responds with a get

# command_moveRel1 = '%s%s%s%s%s' % (msg_ID_moveRel, destinationID_CH1, sourceID, channel_ident) #, Rel_dist)
# command_moveRel2 = '%s%s%s%s%s' % (msg_ID_moveRel, destinationID_CH2, sourceID, channel_ident) #, Rel_dist)

# command_moveRel1 = command_moveRel1.encode('utf-8')
# command_moveRel2 = command_moveRel2.encode('utf-8')

def commandGen_moveRelXY_meth(channel, Rel_dist):
    # move Rel command generator
        
    y = array.array('l', [round(Rel_dist*MLS203_EncCnt_per_mm)])
    # # y.byteswap()
    Rel_dist_hex = y.tostring()
    
    if channel == 1: # x
        destinationID_CH = destinationID_CH1_var
        
    elif channel == 2: # y
        destinationID_CH = destinationID_CH2_var
        
    command_moveRel = msg_ID_moveRel + destinationID_CH + sourceID + channel_ident_long + Rel_dist_hex 
    
    return command_moveRel


## STOP smooth : MGMSG_MOT_MOVE_STOP

msg_ID_stop = b'\x65\x04'

#stop_manner = '\x01' # immediate
stop_manner = b'\x02' # smooth, in a profile way

command_stop1 = msg_ID_stop + channel_ident_short + stop_manner + destinationID_CH1 + sourceID
command_stop2 = msg_ID_stop + channel_ident_short + stop_manner + destinationID_CH2 + sourceID

# you have to READ 6 bytes after this to wait

## resume end_of_move messages # MGMSG_ MOT_RESUME_ENDOFMOVEMSGS

msg_ID_resume_end_of_move_ind1 = b'\x6C\x04\x00\x00' + destinationID_CH1 + sourceID

msg_ID_resume_end_of_move_ind2 = b'\x6C\x04\x00\x00\x22\x01' + destinationID_CH2 + sourceID

## MGMSG_HW_NO_FLASH_PROGRAMMING

#!! make the motor not working, maybe for USB com
msg_ID_noflashprog1 = b'x18\x00\x00\x00' + destinationID_CH1 + sourceID 
msg_ID_noflashprog2 = b'x18\x00\x00\x00' + destinationID_CH2 + sourceID 

## HW read : MGMSG_HW_REQ_INFO
# command = b'\x05\x00\x00\x00\x22\x01'
command_req_info = b'\x05\x00\x00\x00\x11'  + sourceID 
command_req_info1 = b'\x05\x00\x00\x00'+ destinationID_CH1  + sourceID 
command_req_info2 = b'\x05\x00\x00\x00'+ destinationID_CH2  + sourceID 
# motor_stageXY.write(command)
# bb=motor_stageXY.read(90)

# ## trigger usb 0
# 
# msg_ID_trigusb0 = b'x05\x00\x00\x00' + b'\x11' + sourceID 

## MGMSG_MOT_ACK_DCSTATUSUPDATE

# (not used if RS232 conn.) this message called “server alive” must be sent by the server to the controller at least once a second or the controller will stop responding after ~50 commands.

command_serv_alive1 = b'\x92\x04\x00\x00'+ destinationID_CH1  + sourceID 
command_serv_alive2 = b'\x92\x04\x00\x00'+ destinationID_CH2  + sourceID 