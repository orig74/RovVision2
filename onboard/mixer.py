import numpy as np
from numpy import cos,sin


#output dcm from dynamic sim file
#Matrix([
#[cos(q3)*cos(q4(t)), sin(q3(t))*cos(q4(t)), -sin(q4(t))], 
#[-sin(q3)*cos(q5(t)) + sin(q4(t))*sin(q5(t))*cos(q3(t)), sin(q3(t))*sin(q4(t))*sin(q5(t)) + cos(q3(t))*cos(q5(t)), sin(q5(t))*cos(q4(t))], 
#[sin(q3)*sin(q5(t)) + sin(q4(t))*cos(q3(t))*cos(q5(t)), sin(q3(t))*sin(q4(t))*cos(q5(t)) - sin(q5(t))*cos(q3(t)), cos(q4(t))*cos(q5(t))]])
def todcm(yaw,pitch,roll):
    q3,q4,q5=yaw,pitch,roll
    dcm=np.array([
        [cos(q3)*cos(q4), sin(q3)*cos(q4), -sin(q4)],
        [-sin(q3)*cos(q5) + sin(q4)*sin(q5)*cos(q3), sin(q3)*sin(q4)*sin(q5) + cos(q3)*cos(q5), sin(q5)*cos(q4)],
        [sin(q3)*sin(q5) + sin(q4)*cos(q3)*cos(q5), sin(q3)*sin(q4)*cos(q5) - sin(q5)*cos(q3), cos(q4)*cos(q5)]])
    return dcm

def fromdcm(dcm):
    pitch=-np.arcsin(dcm[0,2])
    yaw=np.arctan2(dcm[0,1],dcm[0,0])
    roll=np.arctan2(dcm[1,2],dcm[2,2])
    return np.array([yaw,pitch,roll])

def mix(up_down,left_right,fwd_back,roll,pitch,yaw,pitch_copensate=0.0,roll_copensate=0.0):
    """
    mix all comands to thrusters
    """
    #roll_copensate=0 #for now
    dcm=todcm(0,np.deg2rad(pitch_copensate),np.deg2rad(roll_copensate))
    v=np.array([[fwd_back,left_right,up_down]]).T
    fwd_back,left_right,up_down=(dcm @ v).flatten()
    if abs(pitch_copensate-90)>0.01: ## test for singularity
        v=np.array([roll,pitch,yaw]).reshape(3,1) #rot arround x,y,z
        roll,pitch,yaw=(dcm @ v).flatten() #command in reference frame


    thrusters=np.zeros(8)
    thrusters[:4]+=up_down#/2.0 #/2.0 since thrusters are dircted strait up
    
    thrusters[[4,6]]+=left_right*np.sqrt(2) # sqrt orient in 45 deg compensation
    thrusters[[5,7]]-=left_right*np.sqrt(2)

    thrusters[4:]+=fwd_back*np.sqrt(2)

    thrusters[[0,3]]-=roll
    thrusters[[1,2]]+=roll

    thrusters[[0,1]]+=pitch
    thrusters[[2,3]]-=pitch

    thrusters[[4,7]]+=yaw/2.0 #/2.0 since yaw command using all 4 thrusters  
    thrusters[[5,6]]-=yaw/2.0

    thrusters=np.clip(thrusters,-1,1)

    return list(thrusters) 

if __name__=="__main__":
    from numpy import array as arr
    dcm1=todcm(np.deg2rad(1),np.deg2rad(45),np.deg2rad(00))
    dcm2=todcm(np.deg2rad(0),np.deg2rad(45),np.deg2rad(00))
    #print(dcm2.T @ np.array([[0,0,1]]).T)
    print(dcm1.T @ dcm2 @ arr([[0,1,0]]).T) 
