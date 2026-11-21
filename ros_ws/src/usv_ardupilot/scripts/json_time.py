"""Bound AP's simulated time advance, interpolating between observed physics states.
The transport must acknowledge every output before sending the next step.
"""
import numpy as np
from scipy.spatial.transform import Rotation
class StepLimiter:
    def __init__(self,max_step=.02):
        if not 0<max_step<=.02:raise ValueError('Maximum step must be 0..20 ms')
        self.max_step=max_step;self.last=None;self.interpolated=0;self.max_delta=0.
    def next(self,target):
        if self.last is None:self.last=target;return target
        delta=target['timestamp']-self.last['timestamp']
        if delta < -1e-9:raise RuntimeError('Gazebo reset detected: restart bridge and SITL together')
        if delta<=1e-9:return None
        if delta<=self.max_step+1e-9:result=target
        else:
            a=self.max_step/delta;old=self.last;result=dict(target);result['timestamp']=old['timestamp']+self.max_step
            def mix(x,y):return ((1-a)*np.array(x)+a*np.array(y)).tolist()
            for key in ['position','velocity']:result[key]=mix(old[key],target[key])
            result['imu']={key:mix(old['imu'][key],target['imu'][key]) for key in ['gyro','accel_body']}
            result['battery']={key:(1-a)*old['battery'][key]+a*target['battery'][key] for key in ['voltage','current']}
            def rot(q):return Rotation.from_quat([q[1],q[2],q[3],q[0]])
            r0,r1=rot(old['quaternion']),rot(target['quaternion']);q=(r0*Rotation.from_rotvec((r0.inv()*r1).as_rotvec()*a)).as_quat()
            result['quaternion']=[q[3],*q[:3]];self.interpolated+=1
        self.max_delta=max(self.max_delta,result['timestamp']-self.last['timestamp']);self.last=result;return result
