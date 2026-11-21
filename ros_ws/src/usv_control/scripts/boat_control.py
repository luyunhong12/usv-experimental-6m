#!/usr/bin/env python3
"""ROS payload power gating and bounded single-azimuth velocity/heading control.
Navigation feedback uses IMU + GPS velocity, never the ground-truth odometry.
Direct actuator commands override cmd_vel; heartbeat expiry stops propulsion.
"""
import math
import os
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, MagneticField, Image, CameraInfo, LaserScan, PointCloud2
from geometry_msgs.msg import Twist, Vector3Stamped, TransformStamped
from std_msgs.msg import Float64, Float64MultiArray, String
from tf2_ros import StaticTransformBroadcaster
from gz.transport13 import Node as GzNode
from usv_control.ownership import ControlLease
from gz.msgs10.double_pb2 import Double
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.navsat_pb2 import NavSat

def wrap(v):return math.atan2(math.sin(v),math.cos(v))

class BoatControl(Node):
    def __init__(self):
        super().__init__('boat_control')
        self.declare_parameter('boat_name','usv')
        self.declare_parameter('control_enabled',True)
        self.control_enabled=self.get_parameter('control_enabled').value
        self.declare_parameter('speed_feedforward_m_s',1.63)
        self.speed_scale=float(self.get_parameter('speed_feedforward_m_s').value)
        if self.speed_scale<=0:raise ValueError('Invalid speed scale')
        self.boat=self.get_parameter('boat_name').value
        self.prefix='/'+self.boat
        self.payload_cfg=json.loads((Path(get_package_share_directory('usv_description'))/'config/payloads.json').read_text())
        self.lease=ControlLease(self.boat,'ROS cmd_vel/direct actuators') if self.control_enabled else None
        self.gz=GzNode()
        self.throttle=self.gz.advertise(self.prefix+'/propulsion/throttle',Double)
        self.steering=self.gz.advertise(self.prefix+'/steering/angle',Double)
        self.energy=None;self.energy_wall=0;self.sim=0.;self.actuators=[]
        self.actuator_wall=0.;self.physics_wall=0.;self.health_wall=0.;self.command_sequence=0;self.command_mode="idle";self.fixed_throttle=None;self.transport_retry_wall=time.monotonic();self.transport_retries=0
        self.gz.subscribe(Double_V,self.prefix+'/energy/truth',self.on_energy)
        self.gz.subscribe(Double_V,self.prefix+'/actuators/state',self.on_actuators)
        self.gz.subscribe(Double_V,self.prefix+'/propulsion/state',self.on_physics)
        self.yaw=None;self.rate=0.;self.velocity=None;self.imu_wall=0.;self.gps_wall=0.;self.imu_stamp=-1e9;self.gps_stamp=-1e9
        self.desired=None;self.command_wall=0.;self.hold_yaw=None;self.integral=0.;self.last_sim=None
        self.subscriptions_keep=[];self.publishers_keep=[]
        self.status=self.create_publisher(Float64MultiArray,self.prefix+'/control/state',10)
        self.health=self.create_publisher(String,self.prefix+'/control/health',10)
        payloads=[('imu',Imu,'sensors/imu/imu/data'),('gps',NavSatFix,'sensors/gps/gps/fix'),
                  ('magnetometer',MagneticField,'sensors/magnetometer/data'),
                  ('camera',Image,'sensors/cameras/camera/image_raw'),
                  ('camera_info',CameraInfo,'sensors/cameras/camera/camera_info'),
                  ('lidar',LaserScan,'sensors/lidars/lidar/scan'),
                  ('gps_velocity',Vector3Stamped,'sensors/gps/velocity')]
        cfg=json.loads((Path(get_package_share_directory('usv_description'))/'config/payloads.json').read_text())
        if cfg['advanced_sensors']:
            for n in ['camera_right','camera_side']:
                payloads.extend([(n,Image,'sensors/cameras/'+n+'/image_raw'),(n+'_info',CameraInfo,'sensors/cameras/'+n+'/camera_info')])
            payloads.append(('lidar_points',PointCloud2,'sensors/lidars/lidar/points'))
        for name,kind,topic in payloads:
            pub=self.create_publisher(kind,self.prefix+'/'+topic,qos_profile_sensor_data)
            self.publishers_keep.append(pub)
            self.subscriptions_keep.append(self.create_subscription(kind,self.prefix+'/raw/'+name,
                lambda msg,pub=pub,name=name:self.payload(name,msg,pub),qos_profile_sensor_data))
        self.subscriptions_keep.append(self.create_subscription(Twist,self.prefix+'/cmd_vel',self.cmd_vel,10))
        self.subscriptions_keep.append(self.create_subscription(Float64,self.prefix+'/actuators/throttle',self.direct_throttle,10))
        self.subscriptions_keep.append(self.create_subscription(Float64,self.prefix+'/actuators/steering',self.direct_steering,10))
        self.subscriptions_keep.append(self.create_subscription(Float64,self.prefix+'/actuators/heading_hold_throttle',self.fixed_heading_throttle,10))
        self.gps_velocity_pub=self.create_publisher(Vector3Stamped,self.prefix+"/raw/gps_velocity",qos_profile_sensor_data)
        self.gz.subscribe(NavSat,self.prefix+"/sensors/gps",self.gps_velocity)
        self.static_tf=StaticTransformBroadcaster(self)
        transforms=[]
        cfg=json.loads((Path(get_package_share_directory('usv_description'))/'config/payloads.json').read_text())
        poses={k:v for k,v in cfg['poses'].items() if cfg['advanced_sensors'] or k not in ['camera_right','camera_side']}
        for name,(x,y,z,r,p,yaw) in poses.items():
            t=TransformStamped();t.header.frame_id=self.boat+'/base_link';t.child_frame_id=self.boat+'/'+name
            t.transform.translation.x=float(x);t.transform.translation.y=float(y);t.transform.translation.z=float(z)
            q=Rotation.from_euler('xyz',[r,p,yaw]).as_quat();t.transform.rotation.x=float(q[0]);t.transform.rotation.y=float(q[1]);t.transform.rotation.z=float(q[2]);t.transform.rotation.w=float(q[3]);transforms.append(t)
        t=TransformStamped();t.header.frame_id=self.boat+'/camera';t.child_frame_id=self.boat+'/camera_optical'
        t.transform.rotation.x=-.5;t.transform.rotation.y=.5;t.transform.rotation.z=-.5;t.transform.rotation.w=.5;transforms.append(t)
        if cfg['advanced_sensors']:
            for n in ['camera_right','camera_side']:
                t=TransformStamped();t.header.frame_id=self.boat+'/'+n;t.child_frame_id=self.boat+'/'+n+'_optical';t.transform.rotation.x=-.5;t.transform.rotation.y=.5;t.transform.rotation.z=-.5;t.transform.rotation.w=.5;transforms.append(t)
        self.static_tf.sendTransform(transforms)
        self.watchdog_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=self.create_timer(.05,self.tick,clock=self.watchdog_clock)
    def gps_velocity(self,msg):
        out=Vector3Stamped();out.header.stamp.sec=msg.header.stamp.sec;out.header.stamp.nanosec=msg.header.stamp.nsec
        out.header.frame_id="world";out.vector.x=msg.velocity_east;out.vector.y=msg.velocity_north;out.vector.z=msg.velocity_up
        self.gps_velocity_pub.publish(out)
    def on_energy(self,msg):
        if len(msg.data)>=25 and all(map(math.isfinite,msg.data)):
            self.energy=list(msg.data);self.sim=msg.data[0];self.energy_wall=time.monotonic()
    def on_actuators(self,msg):
        if len(msg.data)>=8 and all(map(math.isfinite,msg.data)):
            self.actuators=list(msg.data);self.actuator_wall=time.monotonic()
    def on_physics(self,msg):
        if len(msg.data)>=17 and all(map(math.isfinite,msg.data)):
            self.physics_wall=time.monotonic()
    def actuator_ready(self):
        now=time.monotonic()
        return self.powered() and now-self.actuator_wall<1 and now-self.physics_wall<1
    def publish_health(self):
        now=time.monotonic()
        if now-self.health_wall<.2:return
        self.health_wall=now
        checks={'control_enabled':self.control_enabled,'energy_available':self.powered(),
                'actuator_feedback':now-self.actuator_wall<1,'physics_feedback':now-self.physics_wall<1,
                'navigation_fresh':self.navigation_fresh()}
        data={'gz_partition':os.environ.get('GZ_PARTITION','default'),'sim_s':self.sim,'ready':all(checks.values()),'checks':checks,
              'missing':[k for k,v in checks.items() if not v],
              'command_sequence':self.command_sequence,'command_mode':self.command_mode,'transport_retries':self.transport_retries,'fixed_throttle_request':self.fixed_throttle if self.command_mode=='fixed_heading' else None,'heading_reference_rad':self.hold_yaw}
        self.health.publish(String(data=json.dumps(data)))
    def powered(self):
        return self.energy is not None and time.monotonic()-self.energy_wall<2 and self.energy[23]<1e-3
    def navigation_fresh(self):
        return (self.yaw is not None and self.velocity is not None
                and time.monotonic()-min(self.imu_wall,self.gps_wall)<1
                and -.2<=self.sim-self.imu_stamp<.5 and -.2<=self.sim-self.gps_stamp<.5)
    def payload(self,name,msg,pub):
        if not self.powered():return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if name in ['imu','gps_velocity'] and not -.2<=self.sim-stamp<.5:return
        if name=='imu':
            q=msg.orientation
            self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.rate=msg.angular_velocity.z;self.imu_wall=time.monotonic();self.imu_stamp=stamp
        elif name=='gps_velocity':
            self.velocity=(msg.vector.x,msg.vector.y);self.gps_wall=time.monotonic();self.gps_stamp=stamp
        if hasattr(msg,'header'):
            frame=(name.removesuffix('_info')+'_optical') if name.startswith('camera') else ('lidar' if name=='lidar_points' else name)
            msg.header.frame_id=self.boat+'/'+frame
            if name=='gps_velocity':msg.header.frame_id='world'
        if name=='camera_right_info':msg.p[3]=-msg.k[0]*self.payload_cfg['camera']['stereo_baseline_m']
        pub.publish(msg)
    def cmd_vel(self,msg):
        if not self.control_enabled:return
        if not all(map(math.isfinite,[msg.linear.x,msg.angular.z])):return
        if not self.navigation_fresh() or not self.actuator_ready():
            self.desired=None;self.hold_yaw=None;self.integral=0.
            self.throttle.publish(Double(data=0.));return
        first=self.desired is None or time.monotonic()-self.command_wall>1
        self.desired=(max(-4.5,min(4.5,msg.linear.x)),max(-.5,min(.5,msg.angular.z)))
        if first:self.hold_yaw=self.yaw;self.integral=0.
        self.command_wall=time.monotonic();self.command_sequence+=1;self.command_mode="velocity";self.fixed_throttle=None
    def fixed_heading_throttle(self,msg):
        if not self.control_enabled or not math.isfinite(msg.data):return
        if not self.navigation_fresh() or not self.actuator_ready():
            self.desired=None;self.throttle.publish(Double(data=0.));return
        if self.desired is None or self.command_mode!='fixed_heading' or time.monotonic()-self.command_wall>1:
            self.hold_yaw=self.yaw;self.integral=0.
        self.fixed_throttle=max(-1.,min(1.,msg.data));self.desired=(math.copysign(1.,msg.data) if abs(msg.data)>.05 else 0.,0.)
        self.command_wall=time.monotonic();self.command_sequence+=1;self.command_mode='fixed_heading'
    def reconnect_stale_transport(self):
        now=time.monotonic()
        if now-self.transport_retry_wall<3:return
        self.transport_retry_wall=now
        for topic,kind,callback,stamp in [('/energy/truth',Double_V,self.on_energy,self.energy_wall),('/actuators/state',Double_V,self.on_actuators,self.actuator_wall),('/propulsion/state',Double_V,self.on_physics,self.physics_wall)]:
            if now-stamp>3:
                self.gz.unsubscribe(self.prefix+topic);self.gz.subscribe(kind,self.prefix+topic,callback);self.transport_retries+=1
    def direct_throttle(self,msg):
        if not self.control_enabled:return
        if not math.isfinite(msg.data):return
        self.desired=None;self.integral=0.;self.command_sequence+=1;self.command_mode="direct";self.fixed_throttle=None
        self.throttle.publish(Double(data=max(-1.,min(1.,msg.data)) if self.actuator_ready() else 0.))
    def direct_steering(self,msg):
        if not self.control_enabled:return
        if not math.isfinite(msg.data):return
        self.desired=None
        if self.actuator_ready():self.steering.publish(Double(data=max(-math.radians(35),min(math.radians(35),msg.data))))
    def tick(self):
        self.reconnect_stale_transport()
        self.publish_health()
        dt=0 if self.last_sim is None else self.sim-self.last_sim;self.last_sim=self.sim
        if dt<0:self.desired=None;self.integral=0.;self.hold_yaw=None
        if self.desired is None:return
        if dt>.5:
            # A clock/transport gap invalidates the old heading reference.
            self.hold_yaw=None;self.integral=0.
        fresh=self.navigation_fresh()
        if not self.actuator_ready() or not fresh or time.monotonic()-self.command_wall>1:
            self.throttle.publish(Double(data=0.));self.desired=None;self.hold_yaw=None;self.integral=0.;return
        if dt<=0:return
        dt=min(dt,.2);speed,turn=self.desired
        if abs(speed)<.02:
            self.throttle.publish(Double(data=0.));self.steering.publish(Double(data=0.));self.integral=0.;self.hold_yaw=self.yaw;return
        actual=self.velocity[0]*math.cos(self.yaw)+self.velocity[1]*math.sin(self.yaw)
        if self.command_mode=='fixed_heading':
            # Steering closes the heading loop; shaft-speed request stays fixed.
            throttle=self.fixed_throttle;raw=abs(throttle)
        else:
            error=abs(speed)-math.copysign(1,speed)*actual
            candidate=max(-.25,min(.35,self.integral+.08*error*dt))
            raw=.05+.95*abs(speed)/self.speed_scale+.35*error+candidate
            if 0<raw<1 or (raw>=1 and error<0) or (raw<=0 and error>0):self.integral=candidate
            throttle=math.copysign(max(0.,min(1.,raw)),speed)
        if abs(turn)<1e-5:
            if self.hold_yaw is None:self.hold_yaw=self.yaw
            heading_error=wrap(self.hold_yaw-self.yaw)
            requested=max(-.3,min(.3,.8*heading_error))
        else:
            heading_error=0.;requested=turn;self.hold_yaw=self.yaw
        angle=-math.copysign(1,speed)*(1.5*(requested-self.rate))
        angle=max(-math.radians(35),min(math.radians(35),angle))
        self.throttle.publish(Double(data=throttle));self.steering.publish(Double(data=angle))
        if self.command_mode=="velocity":self.status.publish(Float64MultiArray(data=[self.sim,speed,actual,heading_error,self.rate,throttle,angle,float(raw>=1)]))
    def close(self):
        if self.control_enabled:self.throttle.publish(Double(data=0.))
        if self.lease:self.lease.close()
        self.gz.unsubscribe(self.prefix+'/sensors/gps');self.gz.unsubscribe(self.prefix+'/energy/truth');self.gz.unsubscribe(self.prefix+'/actuators/state');self.gz.unsubscribe(self.prefix+'/propulsion/state')

def main():
    rclpy.init();node=BoatControl()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        # launch may signal both the process group and the child during cleanup.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
