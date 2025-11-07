#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, Spawn, Kill
from turtlesim.msg import Pose
import math

class FullMapCleaner(Node):
    def __init__(self):
        super().__init__('full_map_cleaner')
        self.pubs = [
            self.create_publisher(Twist, f'/turtle{i}/cmd_vel', 10)
            for i in range(1,5)
        ]
        self.poses = [None]*4
        self.subs = [
            self.create_subscription(Pose, f'/turtle{i}/pose', lambda msg,i=i: self.poses.__setitem__(i-1,msg),10)
            for i in range(1,5)
        ]
        self.timer = self.create_timer(0.1, self.loop)
        self.states = [
            {"moving": True, "row":0, "complete":False} for _ in range(4)
        ]
        self.limits = [
            {"x_min":5.5,"x_max":11.0,"y_min":5.5,"y_max":11.0,"inc":0.7},
            {"x_min":0.0,"x_max":5.5,"y_min":5.5,"y_max":11.0,"inc":0.7},
            {"x_min":5.5,"x_max":11.0,"y_min":0.0,"y_max":5.5,"inc":0.7},
            {"x_min":0.0,"x_max":5.5,"y_min":0.0,"y_max":5.5,"inc":0.7}
        ]
        self.max_rows = [
            int((l["y_max"]-l["y_min"])/l["inc"]) if i%2==0 else int((l["x_max"]-l["x_min"])/l["inc"])
            for i,l in enumerate(self.limits)
        ]
        self.spawn_and_position()
        
    def spawn_and_position(self):
        self.kill('turtle1')
        self.spawn('turtle1',5.5,5.5,0.0)
        self.spawn('turtle2',5.5,5.5,math.pi/2)
        self.spawn('turtle3',5.5,5.5,0.0)
        self.spawn('turtle4',5.5,5.5,math.pi/2)
        self.teleport('turtle1', self.limits[0]["x_min"], self.limits[0]["y_max"],0.0)
        self.teleport('turtle2',5.5,self.limits[1]["y_min"],math.pi/2)
        self.teleport('turtle3',self.limits[2]["x_max"],self.limits[2]["y_min"],math.pi)
        self.teleport('turtle4',0.0,self.limits[3]["y_min"],math.pi/2)
        self.get_logger().info("FULL MAP CLEANER STARTED!")

    def kill(self,name):
        client=self.create_client(Kill,'/kill')
        while not client.wait_for_service(1.0):
            pass
        req=Kill.Request()
        req.name=name
        client.call_async(req)

    def spawn(self,name,x,y,theta):
        client=self.create_client(Spawn,'/spawn')
        while not client.wait_for_service(1.0):
            pass
        req=Spawn.Request()
        req.name,name=req.name,name
        req.x,req.y,req.theta=x,y,theta
        client.call_async(req)

    def teleport(self,name,x,y,theta):
        client=self.create_client(TeleportAbsolute,f'/{name}/teleport_absolute')
        while not client.wait_for_service(1.0):
            pass
        req=TeleportAbsolute.Request()
        req.x,req.y,req.theta=x,y,theta
        client.call_async(req)

    def loop(self):
        if any(p is None for p in self.poses):
            return
        for i in range(4):
            if not self.states[i]["complete"]:
                self.control_turtle(i)
        if all(s["complete"] for s in self.states):
            self.get_logger().info("🎉 ALL QUADRANTS COMPLETELY CLEANED! 🎉")
            self.timer.cancel()

    def control_turtle(self,i):
        p=self.poses[i]
        s=self.states[i]
        l=self.limits[i]
        msg=Twist()
        if i%2==0:
            target_x=l["x_max"] if s["moving"] else l["x_min"]
            target_y=l["y_max"]-(s["row"]*l["inc"])
            if (p.x>=l["x_max"]-0.2 and s["moving"]) or (p.x<=l["x_min"]+0.2 and not s["moving"]):
                if s["row"]>=self.max_rows[i]:
                    self.pubs[i].publish(Twist())
                    s["complete"]=True
                    self.get_logger().info(f"✅ Turtle{i+1}: Quadrant COMPLETE!")
                    return
                s["row"]+=1
                s["moving"]=not s["moving"]
        else:
            target_x=l["x_min"]+(s["row"]*l["inc"])
            target_y=l["y_max"] if s["moving"] else l["y_min"]
            if (p.y>=l["y_max"]-0.2 and s["moving"]) or (p.y<=l["y_min"]+0.2 and not s["moving"]):
                if s["row"]>=self.max_rows[i]:
                    self.pubs[i].publish(Twist())
                    s["complete"]=True
                    self.get_logger().info(f"✅ Turtle{i+1}: Quadrant COMPLETE!")
                    return
                s["row"]+=1
                s["moving"]=not s["moving"]
        angle_err=self.angle_err(p.x,p.y,p.theta,target_x,target_y)
        dist_err=math.sqrt((target_x-p.x)**2+(target_y-p.y)**2)
        msg.linear.x=min(2.0,dist_err*1.5)
        msg.angular.z=4.0*angle_err
        self.pubs[i].publish(msg)

    def angle_err(self,x,y,theta,tx,ty):
        a=math.atan2(ty-y,tx-x)-theta
        while a>math.pi: a-=2*math.pi
        while a<-math.pi: a+=2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node=FullMapCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
