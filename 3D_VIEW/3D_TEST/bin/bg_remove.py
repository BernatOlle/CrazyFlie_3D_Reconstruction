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
'''import cflib.crtp'''
'''from crazyflie_lib_python.cflib.crazyflie.swarm import CachedCfFactory
from crazyflie_lib_python.cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger'''
import multiprocessing as mp
import matplotlib.pyplot as plt
import time
import copy
import math
import threading
import matplotlib
'''from wifi_img_streamer.opencv_viewer_AP import AIDECK'''

matplotlib.use("TkAgg")
import numpy as np
'''from crazyflie_lib_python.cflib.crazyflie.swarm import Swarm'''


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
        URI2: [],

    }

    def wait_for_param_download(self, scf):
        print(scf[1])
        while not scf.cf.param.is_updated:
            time.sleep(1.0)
        print("Parameters downloaded for", scf.cf.link_uri)

    def get_estimated_position(self, scf):
        log_config = LogConfig(name="stateEstimate", period_in_ms=10)
        log_config.add_variable("stateEstimate.x", "float")
        log_config.add_variable("stateEstimate.y", "float")
        log_config.add_variable("stateEstimate.z", "float")
        log_config.add_variable("stateEstimate.yaw", "float")
        log_config.add_variable("stateEstimate.pitch", "float")
        log_config.add_variable("stateEstimate.roll", "float")

        with SyncLogger(scf, log_config) as logger:
            for entry in logger:
                x = entry[1]["stateEstimate.x"]
                y = entry[1]["stateEstimate.y"]
                z = entry[1]["stateEstimate.z"]
                yaw = entry[1]["stateEstimate.yaw"]
                pitch = entry[1]["stateEstimate.pitch"]
                roll = entry[1]["stateEstimate.roll"]
                positions = [x, y, z, yaw]
                angles = [yaw, pitch, roll]
                break
        return angles,positions

    def has_reached_position(self, scf, position):
        print("Recalculating", position[0], position[1], position[2])
        _,current_position = self.get_estimated_position(scf)
        threshold_distance = 0.15

        distance = (
            (current_position[0] - position[0]) ** 2
            + (current_position[1] - position[1]) ** 2
            + (current_position[2] - position[2]) ** 2
        ) ** 0.5

        return distance <= threshold_distance

    def take_off(self, scf, init_value, z):
        cf = scf.cf
        uri = scf._link_uri
        take_off_time = 1.0
        sleep_time = 0.1
        steps = int(take_off_time / sleep_time)

        vz = z / take_off_time

        print(vz)

        if uri == self.URI2:
            yaw = 180  # take_off
        else:
            yaw = 0

        for i in range(steps):
            _, curr_x = self.get_estimated_position(scf)
            print("Pos",curr_x)
            print("dif_yaw",yaw - abs(curr_x[3]))
            if (
                yaw - abs(curr_x[3])
            ) < 15:  # si el dron ja ha fet la mitja volta que no giri mes
                cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
            elif((curr_x[3]>0)):
                cf.commander.send_velocity_world_setpoint(0, 0, vz, -yaw)
            elif((curr_x[3]<0)):
                cf.commander.send_velocity_world_setpoint(0, 0, vz, yaw)
            time.sleep(sleep_time)

        for i in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, z)
            time.sleep(0.1)

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

    def go_to(self, scf, sequence):
        cf = scf.cf
        uri = scf._link_uri

        for position in sequence:
            print("Setting position " + uri + " {}".format(position))
            for i in range(5):
                cf.commander.send_position_setpoint(
                    position[0], position[1], position[2], position[3]
                )
                time.sleep(0.1)

            """while not self.has_reached_position(scf, position):
                curr_pos = self.get_estimated_position(scf)
                time_to_go = 1
                dif_x = position[0] - curr_pos[0]
                dif_y = position[1] - curr_pos[1]
                dif_z = position[2] - curr_pos[2]
                dif_yaw = position[3] - curr_pos[3]

                cf.commander.send_hover_setpoint(
                    dif_x / time_to_go,
                    dif_y / time_to_go,
                    dif_yaw / time_to_go,
                    position[2],
                )
                time.sleep(0.1)"""

            for i in range(25):
                cf.commander.send_hover_setpoint(0, 0, 0, position[2])
                time.sleep(0.1)

    def sequence_point(self,n_points,distance,z,degrees):
        n_points+=1
        theta = np.linspace(np.radians(degrees), 2*np.pi, n_points, endpoint=True)
        x = distance * np.cos(theta+np.pi)
        y = distance * np.sin(theta+np.pi)
        z_v = [z]*n_points
        grados = np.degrees(theta)
        sequence = list(zip(x, y, z_v, grados))
        return sequence
    
    def do_next_point(self,seq,i, stop_event):

        with Swarm(self.uris, factory=self.factory) as swarm:
            self.pos = i
            self.sequence = seq
            self.stop_event = stop_event
            swarm.parallel(self.next_point, args_dict=self.seq_args)
    
    def do_stop_event(self,stop_event):
        print("Change event")
        self.stop_event = stop_event

    def next_point(self,seq,i, stop_event):
        self.sequence = seq
        self.stop_event = stop_event
        '''cf = scf.cf
        uri = scf._link_uri'''
        position = self.sequence[0]
        print('Setting position '+"uri"+' {}'.format(position))
        for i  in range(5):
            '''cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2], 
                                                position[3])'''
            time.sleep(0.1)

        while not self.stop_event.is_set():
            print("Wait until event",self.stop_event)
            '''cf.commander.send_hover_setpoint(0, 0, 0, position[2])'''
            time.sleep(0.1)

    
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

    def init_dron(self):
        cflib.crtp.init_drivers()
        
        self.factory = CachedCfFactory(rw_cache="./cache")
        self.continue_running = True
        with Swarm(self.uris, factory=self.factory) as swarm:
            swarm.reset_estimators()

            print("Waiting for parameters to be downloaded...")
            swarm.parallel(self.wait_for_param_download)
            self.init_values = swarm.get_estimated_positions()

            zdistance = [0.6]  # radio, npoints, cicle seconds, photo seconds , z distance

            for uri in self.uris:
                self.seq_args[uri].append(self.init_values[uri])
                self.seq_args[uri].extend(zdistance)

    def do_take_off(self):
        cflib.crtp.init_drivers()
        self.factory = CachedCfFactory(rw_cache="./cache")
        with Swarm(self.uris, factory=self.factory) as swarm:
            print("Take off")
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


dron = Dron()
n_points = 10
#cam = AIDECK()
yes = True
seq = dron.sequence_point(n_points,0.6,0.6,0)
print(seq)
'''if yes:
    dron.init_dron() 
    dron.do_take_off()  '''

    

for i in range(n_points):    
    stop_event = threading.Event()
    if yes:
        thread = threading.Thread(target=dron.next_point, args=(seq,0,stop_event))
        thread.start()
    #cam.take_photo()
    time.sleep(2)
    if yes:
       stop_event.set()
       dron.do_stop_event(stop_event)
       thread.join() 
    point = seq[1]
    n_points -= 1
    seq = dron.sequence_point(n_points,0.6,0.6,point[3])
    
'''if yes:
  dron.do_land() '''



#TODO
#Change the stop_event variable every time and change the firmware to be able to send the picture of that moment and not from the buffer