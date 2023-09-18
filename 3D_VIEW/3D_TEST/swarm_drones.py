"""
Version of the AutonomousSequence.py example connecting to 10 Crazyflies.
The Crazyflies go straight up, hover a while and land but the code is fairly
generic and each Crazyflie has its own sequence of setpoints that it files
to.
The layout of the positions:
    x2      x1      x0
y3  10              4
            ^ Y
            |
y2  9       6       3
            |
            +------> X

y1  8       5       2


y0  7               1
"""
import time
import cflib.crtp
from crazyflie_lib_python.cflib.crazyflie.swarm import CachedCfFactory
from crazyflie_lib_python.cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
import multiprocessing as mp
import matplotlib.pyplot as plt
import time
import copy
import math
import threading
import matplotlib
from opencv_viewer import AIDECK
import pickle

matplotlib.use("TkAgg")
import numpy as np
from crazyflie_lib_python.cflib.crazyflie.swarm import Swarm


class Dron:
    # Change uris and sequences according to your setup
    URI1 = "radio://0/100/2M/E7E7E7E701"
    URI2 = "radio://0/100/2M/E7E7E7E702"
    URI3 = "radio://0/80/2M/E7E7E7E703"
    #    x   y   z  time
    

    # List of URIs, comment the one you do not want to fly
    uris = {
            URI3, 
            #URI2
             }
    
    seq_args = {
        URI3: [],
        #URI2: [],

    }

    def wait_for_param_download(self, scf):
        print(scf[1])
        while not scf.cf.param.is_updated:
            time.sleep(1.0)
        print("Parameters downloaded for", scf.cf.link_uri)

    def interpolate(self,x, x1, y1, x2, y2):
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)

    def get_battery(self,desired_voltage):
        voltages = [3.00, 3.78, 3.83, 3.87, 3.89, 3.92, 3.96, 4.00, 4.04, 4.10]
        percentages = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90]

        for i in range(len(voltages) - 1):
            if voltages[i] <= desired_voltage <= voltages[i + 1]:
                interpolated_percentage = self.interpolate(desired_voltage, voltages[i], percentages[i], voltages[i + 1], percentages[i + 1])
                return interpolated_percentage

    def get_estimated_position(self, scf):
        cf = scf.cf
        log_config = LogConfig(name="stateEstimate", period_in_ms=10)
        log_config.add_variable("stateEstimate.x", "float")
        log_config.add_variable("stateEstimate.y", "float")
        log_config.add_variable("stateEstimate.z", "float")
        log_config.add_variable("stateEstimate.yaw", "float")
        log_config.add_variable("stateEstimate.pitch", "float")
        log_config.add_variable("stateEstimate.roll", "float")
        log_config.add_variable('pm.vbat', 'FP16')


        with SyncLogger(scf, log_config) as logger:
            for entry in logger:
                x = entry[1]["stateEstimate.x"]
                y = entry[1]["stateEstimate.y"]
                z = entry[1]["stateEstimate.z"]
                yaw = entry[1]["stateEstimate.yaw"]
                pitch = entry[1]["stateEstimate.pitch"]
                roll = entry[1]["stateEstimate.roll"]
                battery = entry[1]['pm.vbat']
                positions = [x, y, z, yaw]
                angles = [yaw, pitch, roll]
                self.battery = battery
                break
        return angles,positions 
    
    

    
    def get_cfs(self, uri):
        with Swarm(self.uris, factory=self.factory) as swarm:
            return swarm._cfs[uri]


    def take_off(self, scf, init_value, z):
        cf = scf.cf
        uri = scf._link_uri
        take_off_time = 3.0
        sleep_time = 0.1
        steps = int(take_off_time / sleep_time)

        

        vz = self.z_distance / take_off_time

        print(vz)
        yaw = self.prev_angle

        yaw_rate = yaw/take_off_time
        
        _, curr_x = self.get_estimated_position(scf)
        print(self.battery)
        self.previous_battery = self.battery
        if (self.battery > 3.4):
            for i in range(steps):
                    _, curr_x = self.get_estimated_position(scf)
                    if curr_x[3] < 0:
                        angle = 360 - abs(curr_x[3])
                    else:
                        angle = curr_x[3]
                    print("Pos",curr_x)
                    print("dif_yaw",abs(yaw - angle))
                    if (
                        abs(yaw - angle)
                    ) < 15:  # si el dron ja ha fet la mitja volta que no giri mes
                        
                        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
                    elif((angle>yaw)):
                        cf.commander.send_velocity_world_setpoint(0, 0, vz, -yaw_rate)
                    elif((angle<yaw)):
                        cf.commander.send_velocity_world_setpoint(0, 0, vz, yaw_rate)
                    time.sleep(sleep_time)

            for i in range(6):
                cf.commander.send_hover_setpoint(0, 0, 0, self.z_distance)
                time.sleep(0.1)
        else:
            print("Low battery set")
            self.low_battery = True

    def collision_avoidance(self, aux_drones):
        for scf in aux_drones:
            _,pos_drones = self.get_estimated_position(scf)
            # Falta completar

    def land(self, scf, init_value, z):
        cf = scf.cf
        uri = scf._link_uri
        # last_points = self.get_estimated_position(scf)

        # dis_vec = [v1 - v2 for v1, v2 in zip(init_values, last_points)]
        print("Init_points", init_value)
        # print("Last_points",last_points)

        landing_time = 4.0
        sleep_time = 0.1
        steps = int(landing_time / sleep_time)
        dist = init_value[2]
        print(dist)
        vz = -z / landing_time

        print(vz)

        if uri == self.URI2:
            yaw = 180  # land
        else:
            yaw = 0

        for i in range(10):
            cf.commander.send_position_setpoint(init_value[0], init_value[1], z, yaw)
            time.sleep(0.1)

        for i in range(steps):
            _,curr_x = self.get_estimated_position(scf)
            print("Pos_land",curr_x)
            print("dif_yaw",yaw - abs(curr_x[3]))
            if (
                yaw - abs(curr_x[3])
            ) > 170:  # si el dron ja ha fet la mitja volta que no giri mes
                cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
            elif((curr_x[3]>0)):
                cf.commander.send_velocity_world_setpoint(0, 0, vz, yaw)
            elif((curr_x[3]<0)):
                cf.commander.send_velocity_world_setpoint(0, 0, vz, -yaw)

            time.sleep(sleep_time)

        cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)


    def land_low_battery(self, scf, z):
        cf = scf.cf
        uri = scf._link_uri
        # last_points = self.get_estimated_position(scf)

        # dis_vec = [v1 - v2 for v1, v2 in zip(init_values, last_points)]
        print("Landing Battery")
        landing_time = 4.0
        sleep_time = 0.1
        steps = int(landing_time / sleep_time)
       
        vz = -z / landing_time

        print(vz)


        for i in range(steps):
            _,curr_x = self.get_estimated_position(scf)
            print("Pos_land",curr_x)
            cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)

            time.sleep(sleep_time)

        cf.commander.send_stop_setpoint()
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)


    def sequence_point(self,n_points,distance,z,degrees, final_degrees):
        n_points+=1
        theta = np.linspace(np.radians(degrees), np.radians(final_degrees), n_points, endpoint=True)
        x = distance * np.cos(theta+np.pi)
        y = distance * np.sin(theta+np.pi)*1
        z_v = [z]*n_points
        '''+self.init_z'''
        grados = np.degrees(theta)
        sequence = list(zip(x, y, z_v, grados))
        return sequence
    
    def has_reached_position(self,current_position, position):
        threshold_distance = 0.1

        distance = ((current_position[0] - position[0]) ** 2 +
                    (current_position[1] - position[1]) ** 2 +
                    (current_position[2] - position[2]) ** 2) ** 0.5

        return distance <= threshold_distance
    
    def do_next_point(self,seq,i, stop_event, get_pos):
        

        with Swarm(self.uris, factory=self.factory) as swarm:
            _,_,self.battery = swarm.get_estimated_positions()
            print("Battery",self.battery)
            for uri in self.uris:
                if self.battery[uri] < 3.15 and (self.previous_battery-self.battery[uri])<0.1:
                    self.low_battery = True
                    print("Low battery set")
            self.pos = i
            self.sequence = seq
            self.stop_event = stop_event
            self.get_pos = get_pos
            self.previous_battery = self.battery[uri]
            swarm.parallel(self.next_point, args_dict=self.seq_args)

    
    
    def do_stop_event(self,stop_event):
        print("Change event")
        self.stop_event = stop_event

    def do_get_pos(self,get_pos):
        self.get_pos = get_pos
    
    def do_all_seq(self,seq):
        self.at_point = False
        with Swarm(self.uris, factory=self.factory) as swarm:
            self.act_seq = seq
            print("do all seq")
            swarm.parallel(self.all_seq, args_dict=self.seq_args)

    def all_seq(self,scf, init_value, z):
        cf = scf.cf
        uri = scf._link_uri
        

        print("Go to the last position")
        for position in self.act_seq:
                print('Setting position '+uri+' {}'.format(position))
                for i  in range(5):
                    cf.commander.send_position_setpoint(position[0],
                                                        position[1],
                                                        position[2], 
                                                        position[3])
                    time.sleep(0.05)

        self.at_point = True
        
        return position


    def next_point(self, scf, init_value, z):
        print("ini")
        k = 0
        cf = scf.cf
        uri = scf._link_uri
        position = self.sequence[self.pos]
        print("Getting position")
        _,pos = self.get_estimated_position(scf)
        #print(self.battery)
        self.actual_pos = position

        if not self.low_battery:
            print('Setting position at '+uri+' {}'.format(position))
            while not(self.has_reached_position(pos,position)):
                cf.commander.send_position_setpoint(position[0],
                                                    position[1],
                                                    position[2], 
                                                    position[3])
                _,pos = self.get_estimated_position(scf)
                print(pos)
                time.sleep(0.1)


            self.drone_at_point = True
            print("Waiting to take the photo")
            while not self.stop_event.is_set():
                if self.get_pos.is_set() and k == 0:
                    print("Get position to take the photo")
                    self.angle, self.position = self.get_estimated_position(scf)
                    k= 1
                if not(self.has_reached_position(pos,position)):
                    print("Recalculating position")
                    cf.commander.send_position_setpoint(position[0],
                                                    position[1],
                                                    position[2], 
                                                    position[3])
                    
                else:
                    cf.commander.send_hover_setpoint(0, 0, 0, position[2])

                _,pos = self.get_estimated_position(scf)
                time.sleep(0.1)
            self.drone_at_point=False
        else:
            self.land_low_battery(scf, position[2])  
    
    def run_sequence(self, scf, sequence, init_value, z):
        try:
            aux_drones = []
            cam = AIDECK()
            """for uri, aux_scf in cfs.items(): 
                aux_drones.append(aux_scf)"""
            uri = scf._link_uri
            cf = scf.cf

            init_value = [init_value.x, init_value.y, init_value.z]
            self.take_off(scf, sequence, init_value, z)
            '''self.go_to(scf, sequence)'''

            sequence = self.sequence_point(20,0.3,1,0)
            
            for position in sequence:
                print('Setting position '+uri+' {}'.format(position))
                for i  in range(5):
                    cf.commander.send_position_setpoint(position[0],
                                                        position[1],
                                                        position[2], 
                                                        position[3])
                    time.sleep(0.1)

                for i  in range(25):
                    cf.commander.send_hover_setpoint(0, 0, 0, 1)
                    time.sleep(0.1)
                #cam.take_photo(800,600)
                for i  in range(10):
                    cf.commander.send_hover_setpoint(0, 0, 0, 1)
                    time.sleep(0.1)

            self.land(scf, sequence, init_value, z)
        except Exception as e:
            print("Error", e)

    def init_dron(self,uri):
        cflib.crtp.init_drivers()
        self.factory = CachedCfFactory(rw_cache="./cache")
        self.drone_at_point=False
        self.low_battery = False
        
        with Swarm(self.uris, factory=self.factory) as swarm:
            swarm.reset_estimators()

            print("Waiting for parameters to be downloaded...")
            swarm.parallel(self.wait_for_param_download)
            self.init_values, self.init_angles,_ = swarm.get_estimated_positions()
            self.init_z = self.init_values[uri][2]
            zdistance = [0.6]  # radio, npoints, cicle seconds, photo seconds , z distance

            self.seq_args[uri].append(self.init_values[uri])
            self.seq_args[uri].extend(zdistance)
         

    def do_take_off(self,z, prev_angle):
        cflib.crtp.init_drivers()
        self.factory = CachedCfFactory(rw_cache="./cache")
        with Swarm(self.uris, factory=self.factory) as swarm:
            print("Take off")
            self.z_distance = z
            self.prev_angle = prev_angle
            swarm.parallel(self.take_off, args_dict=self.seq_args)

    def do_land(self):
        with Swarm(self.uris, factory=self.factory) as swarm:
            print("Land")
            swarm.parallel(self.land, args_dict=self.seq_args)

    def main(self):
        #logging.basicConfig(level=logging.DEBUG)
        cflib.crtp.init_drivers()

        factory = CachedCfFactory(rw_cache="./cache")
        with Swarm(self.uris, factory=factory) as swarm:
            # If the copters are started in their correct positions this is
            # probably not needed. The Kalman filter will have time to converge
            # any way since it takes a while to start them all up and connect. We
            # keep the code here to illustrate how to do it.
            swarm.reset_estimators()

            # The current values of all parameters are downloaded as a part of the
            # connections sequence. Since we have 10 copters this is clogging up
            # communication and we have to wait for it to finish before we start
            # flying.
            print("Waiting for parameters to be downloaded...")
            swarm.parallel(self.wait_for_param_download)
            init_values = swarm.get_estimated_positions()
            zdistance = [1]  # radio, npoints, cicle seconds, photo seconds , z distance

            for uri in self.uris:
                self.seq_args[uri].append(init_values[uri])
                self.seq_args[uri].extend(zdistance)

            swarm.parallel(self.take_off, args_dict=self.seq_args)
            swarm.parallel(self.take_off, args_dict=self.seq_args)

    def get_rotation_matrix(self, angle):
        # Matriz de rotación alrededor del eje Z (yaw)
        yaw, pitch, roll = np.deg2rad(angle[0]), np.deg2rad(angle[1]), np.deg2rad(angle[2])

        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Matriz de rotación alrededor del eje X (pitch)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ])

        # Matriz de rotación alrededor del eje Y (roll)
        R_y = np.array([
            [np.cos(roll), 0, np.sin(roll)],
            [0, 1, 0],
            [-np.sin(roll), 0, np.cos(roll)]
        ])

        # Matriz de rotación total
        R = np.dot(np.dot(R_z, R_x), R_y)

        angle_degrees = -90
        angle_radians = np.radians(angle_degrees)

        cos_theta = np.cos(angle_radians)
        sin_theta = np.sin(angle_radians)

        # Matriz de rotación adicional alrededor del eje z
        additional_rotation = np.array(
            [[cos_theta, -sin_theta, 0], [sin_theta, cos_theta, 0], [0, 0, 1]]
        )

        # Sumar la rotación adicional a la matriz de rotación existente
        R = additional_rotation @ R

        return R

if __name__ == "__main__":
    dron = Dron()
    n_points = 12

    cam = AIDECK("Test_1")
    tran_matrix_final_drone = {}
    tran_matrix = np.zeros((3,1))
    rot_matrix = np.zeros(())

    yes = True
    seq = dron.sequence_point(n_points,0.6,0.6,0) #n_points,distance,z,degrees
    print(seq)
    all_scf = {}
    if yes:
        dron.init_dron() 
        dron.do_take_off()  
        

    for i in range(n_points+1):
        print("Next sequence", i)
        stop_event = threading.Event() 
        get_pos = threading.Event()   
        if yes:
            thread = threading.Thread(target=dron.do_next_point, args=(seq,0,stop_event, get_pos))
            thread.start()
            print("Thread started")
        time.sleep(2)    
        cam.take_photo()
        get_pos.set()
        dron.do_get_pos(get_pos)
        time.sleep(2)
        print("Angle, position", dron.angle, dron.position)
        tran_matrix_drone = np.array(dron.position)[:3].reshape(-1, 1)
        rot_matrix_drone = dron.get_rotation_matrix(dron.angle)

        tran_matrix_final_drone[i] = [tran_matrix_drone, rot_matrix_drone]

        if yes:
            stop_event.set()
            dron.do_stop_event(stop_event)
            print("Event set")
            print("Waiting thread")
            thread.join()
            if not(len(seq) == 1):
                point = seq[1]
                n_points -= 1
                seq = dron.sequence_point(n_points,0.6,0.6,point[3])
                print(seq)

        
        
    if yes:
        dron.do_land() 

    with open("tran_matrix_final_drone.pickle", "wb") as file:
            pickle.dump(tran_matrix_final_drone, file)
            print("Acaba")