import numpy as np
class CycArr():
    def __init__(self,size=20000):
        self.buf=[]
        self.size=size
        self.cnt=0
        self.changed = False

    def add(self,arr):
        self.buf.append(arr)
        if len(self.buf)>self.size:
            self.buf.pop(0)
        self.cnt+=1
        self.changed = True

    def get_last(self,label):
        if len(self.buf)>0:
            return self.buf[-1][label]

    def get_data(self,labels):
        data = np.zeros((len(self.buf),len(labels)))
        for i,d in enumerate(self.buf):
            for j,l in enumerate(labels):
                if l in d:
                    data[i][j]=d[l]
                else:
                    data[i][j]=0
        return data

    def get_vec(self):
        return np.array([d for _,d in self.buf])

    def __len__(self):
        return len(self.buf)

    def reset(self):
        self.buf=[]



