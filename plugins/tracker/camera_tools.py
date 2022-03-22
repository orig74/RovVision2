# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import cv2

#using lef hand coordinate system
#cameras are aligned to x
## http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html


__TR = np.array([\
        [   1,  0,  0,  0,  ],
        [   0,  1,  0,  0,  ],
        [   0,  0,  1,  0,  ]])

def roty(a):
	R_y = \
		np.array([\
			[np.cos(a),    0,      np.sin(a)  ],
            [0,                     1,      0         ],
            [-np.sin(a),   0,      np.cos(a)  ]
                    ])
	return R_y
def rotx(a):
        ca = np.cos(a)
        sa = np.sin(a)

        R_x = \
            np.array([  [   1,  0,  0   ],
                        [   0,  ca, -sa ],
                        [   0,  sa, ca  ],
                        ])
        return R_x

def rotz(a):
    ch = np.cos(a)
    sh = np.sin(a)
    Rz = np.array([
        [   ch,     -sh,    0],
        [   sh,     ch,     0],
        [   0,      0,      1]])
    return Rz

def get_stereo_cameras(f,sz,base_line,pitch_rad=0):
    M = np.array([\
            [   f, 0,  sz[0]/2   ],
            [   0,  f, sz[1]/2   ],
            [   0,  0,  1,  ]])
    proj_caml= M @ rotx(pitch_rad) @ __TR
    T = -rotx(pitch_rad).T @ np.array([[base_line,0,0]]).T
    #print(T)
    proj_camr=M @ np.hstack((rotx(pitch_rad),T))
    #print(proj_caml)
    #print(proj_camr)
    return proj_caml,proj_camr

def get_stereo_cameras_yaw(f,sz,base_line,yaw_rad,pitch_rad=0):
    M = np.array([\
            [   f, 0,  sz[0]/2   ],
            [   0,  f, sz[1]/2   ],
            [   0,  0,  1,  ]])
    #R= rotx(pitch_rad) @ rotz(yaw_rad)
    R= rotz(yaw_rad) @ rotx(pitch_rad)
    proj_caml= M @ R @ __TR
    T = -R.T @ np.array([[base_line,0,0]]).T
    #print(T)
    proj_camr=M @ np.hstack((R,T))
    #print(proj_caml)
    #print(proj_camr)
    return proj_caml,proj_camr


def triangulate(prjl,prjr,xl,yl,xr,yr):
    #x=cv2.triangulatePoints(Pl,Pr,np.array([sz[0]/2,sz[1]/2]),np.array([sz[0]/2+50.0,sz[1]/2]))
    p1 = np.array([xl,yl],dtype=float)
    p2 = np.array([xr,yr],dtype=float)
    #print(p1,p2)
    p=cv2.triangulatePoints(prjl,prjr,p1,p2)
    #print(p)
    p=p/p[3,0]
    x,y,z = p.flatten()[:3]
    ### convert from opencv to our coordinates which is z-up x-forward y-right
    #   opencv coordinate system is described here:  
    #   http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html  
    return z,x,-y
    #return x,y,z



if __name__=='__main__':
    #import sys
    #sys.path.append('../')
    f = 815.3663482560033
    sz=(960,600)
    #sz=(600,960)
    Pl,Pr = get_stereo_cameras(f,sz,0.1,np.radians(0))

    #x=cv2.triangulatePoints(Pl,Pr,np.array([sz[0]/2,sz[1]/2]),np.array([sz[0]/2+50.0,sz[1]/2]))

    #print(triangulate(Pl,Pr,320,256,320-40,256))
    #print(triangulate(Pl,Pr,320,256,320-140,256))
    #print(triangulate(Pl,Pr,320,256,320-240,256))
    #print(triangulate(Pl,Pr,256,320,256,320-40))
    pl=960/2+0
    pr=pl+50
    print(Pl)
    print(Pr)
    print(triangulate(Pl,Pr,pl,300,pr,300))
    print(pl)
    print(triangulate(Pl,Pr,490, 300, 375, 299.0))
