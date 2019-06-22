import numpy as np

def mix(up_down,left_right,fwd_back,roll,pitch,yaw):
    """
    mix all comands to thrusters
    """
    thrusters=np.zeros(8)
    thrusters[:4]+=up_down
    
    thrusters[[4,6]]-=left_right
    thrusters[[5,7]]+=left_right

    thrusters[4:]+=fwd_back

    thrusters[[0,3]]+=roll
    thrusters[[1,2]]-=roll

    thrusters[[0,1]]+=pitch
    thrusters[[2,3]]-=pitch

    thrusters[[4,7]]+=yaw
    thrusters[[5,6]]-=yaw

    thrusters=np.clip(thrusters,-1,1)

    return list(thrusters) 

