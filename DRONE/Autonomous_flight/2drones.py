# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.
This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""
from threading import Thread
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import time
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
import multiprocessing as mp
"""
The Crazyflie module is used to easily connect/send/receive data
from a Crazyflie.
Each function in the Crazyflie has a class in the module that can be used
to access that functionality. The same design is then used in the Crazyflie
firmware which makes the mapping 1:1 in most cases.
"""

"""Representation of one log configuration that enables 
logging from the Crazyflie"""

"""
The synchronous Crazyflie class is a wrapper around the "normal" Crazyflie
class. It handles the asynchronous nature of the Crazyflie API and turns it
into blocking functions. It is useful for simple scripts that performs tasks
as a sequence of events.
Example:
```python
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with PositionHlCommander(scf, default_height=0.5, default_velocity=0.2) as pc:
        # fly onto a landing platform at non-zero height (ex: from floor to desk, etc)
        pc.forward(1.0)
        pc.left(1.0)
```
"""
"""
This class provides synchronous access to log data from the Crazyflie.
It acts as an iterator and returns the next value on each iteration.
If no value is available it blocks until log data is received again.
"""
class TOC:
    def __init__(self, cf):
        self._cf = cf 
        self.log_conf = LogConfig(name='Position', period_in_ms=200)
        self.log_conf.add_variable('kalman.stateX', 'float')
        self.log_conf.add_variable('kalman.stateY', 'float')
        self.log_conf.add_variable('kalman.stateZ', 'float')

        # self.log_conf2 = LogConfig(name='state', period_in_ms=100)
        # self.log_conf2.add_variable('ctrltarget.x', 'float')
        # self.log_conf2.add_variable('ctrltarget.y', 'float')
        # self.log_conf2.add_variable('ctrltarget.z', 'float')

        # self.log_conf2 = LogConfig(name='state', period_in_ms=10)
        # self.log_conf2.add_variable('zranging.offset', 'float')
        # self.log_conf2.add_variable('zranging.history', 'float')
        # self.log_conf2.add_variable('zranging.collect', 'float')


        self._cf.log.add_config(self.log_conf)
        self.log_conf.data_received_cb.add_callback(self.position_callback)
        self.log_conf.start()

        # self._cf.log.add_config(self.log_conf2)
        # self.log_conf2.data_received_cb.add_callback(self.state_callback)
        # self.log_conf2.start()

    def position_callback(self, timestamp, data, logconf):
        self.x = data['kalman.stateX']
        self.y = data['kalman.stateY']
        self.z = data['kalman.stateZ']

    # def state_callback(self, timestamp, data, logconf):
    #     self.offset = data['zranging.offset']
    #     self.history = data['zranging.collect']
    #     self.collect = data['zranging.history']

# URI to the Crazyflie to connect to
uri_link_100 = 'radio://0/100/2M/E7E7E7E7E7'
uri_link_80 = 'radio://0/80/2M/E7E7E7E7E7'

uri = [uri_helper.uri_from_env(default=uri_link_80),uri_helper.uri_from_env(default=uri_link_100)]
# Change the sequence according to your setup
#             x    y    z  YAW





def update_graph(x,y,z,final,fig,i):
    print("Graph")
    print(final.value)
    while final.value == False:
        ax = fig.add_subplot(111, projection='3d')
        # Grafica los puntos como un scatter plot
        x_values = x.value
        y_values = y.value
        z_values = z.value
        
        
        anc_x = [p[0] for p in anchors]
        anc_y = [p[1] for p in anchors]
        anc_z = [p[2] for p in anchors]
        sc = ax.scatter(anc_x*100, anc_y*100, anc_z*100, c='blue')
        
        
        sc_red = ax.scatter(x_values,y_values,z_values, c='red', s =70)
       
        ax.text(x_values, y_values, z_values, str(x_values*100)+' '+str(y_values*100)+' '+ str(z_values*100))

        # Configura los labels de los ejes
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        print("started")
        
        plt.show(block = False)
        plt.pause(0.02)
        plt.clf()
        print(final.value)
        
    plt.close()
    
    
sequence = [[
    
    (-0.25, 0.0, 1, 0),
    (-0.217, -0.125, 1, 30),
    (-0.125, -0.217, 1, 60),
    (0.0, -0.25, 1, 90),
    (0.125, -0.217, 1, 120),
    (0.217, -0.125, 1, 150),
    (0.25, 0.0, 1, 180),
    (0.217, 0.125, 1, 210),
    (0.125, 0.217, 1, 240),
    (0.0, 0.25, 1, 270),
    (-0.125, 0.217, 1, 300),
    (-0.217, 0.125, 1, 330),
    (-0.25, 0.0, 1, 0),
    (-0.25, 0.0, 1, 0),
    (-0.25, 0.0, 1, 0),
    (-0.25, 0.0, 0.7, 0),
    (-0.25, 0.0, 0.35, 0),
    
],[ 
    (-0.25, 0.0, 1.7, 0),
    (0.125, 0.217, 1.7, 240),
    (0.0, 0.25, 1.7, 270),
    (-0.125, 0.217, 1.7, 300),
    (-0.217, 0.125, 1.7, 330),
    (-0.25, 0.0, 1.7, 0),
    (-0.25, 0.0, 1.7, 0),
    (-0.25, 0.0, 1.7, 0),
    (-0.217, -0.125, 1.7, 30),
    (-0.125, -0.217, 1.7, 60),
    (0.0, -0.25, 1.7, 90),
    (0.125, -0.217, 1.7, 120),
    (0.217, -0.125, 1.7, 150),
    (0.25, 0.0, 1.7, 180),
    (0.217, 0.125, 1.7, 210),
    (0.217, 0.125, 1, 0),
    (0.217, 0.125, 0.35, 0),
    
]]



anchors = [
    
    (-1.1, -1.0, 0.1),
    (-1.1, 1.0, 2.2),
    (1.1, 1.0, 0.2),
    (1.1, -1.0, 2.2),
    (-1.1, -1.0, 2.2),
    (-1.1, 1.0, 0.1),
    (1.1, 1.0, 2.2),
    (1.1, -1.0, 0.1),
]

x = [p[0] for p in anchors]
y = [p[1] for p in anchors]
z = [p[2] for p in anchors]

# Crea la figura y el subplot 3D
fig = plt.figure(figsize=(15,15))



x = mp.Value('f', 0)
y = mp.Value('f', 0)
z = mp.Value('f', 0)
index = mp.Value('i',0)
final = mp.Value('b', False)


p = mp.Process(target=update_graph, args=(x,y,z,final,fig,index,))     
#p.start()

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                
                break

def reset_estimator(scf):
    global data
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    data = TOC(cf)
    
    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

    
def run_sequence(scf,sequence):
    cf = scf.cf
    cf.commander.send_velocity_world_setpoint(0.1,0.1,0.1,10)
    cf.mem
    limit = 0.01
    print(data.x,data.y, data.z)
    
    for position in sequence:
        print('Setting position {}'.format(position))
        print('Threshold positions x',position[0]+limit, position[0]-limit)
        
        #while ((data.x > position[0]+limit or data.x < position[0]-limit) and (data.y > position[1]+limit or data.y < position[1]-limit) and (data.z > position[2]+limit or data.z < position[2]-limit)):
        for i in range(100): 
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
            #update_graph(position[x],position[y],position[z])
            print("Drona in position: ",data.x,data.y, data.z)
            x.value = data.x
            y.value = data.y
            z.value = data.z
            time.sleep(0.007)
            index.value += 1
        index.value = 0
        print("NextPoint")
        time.sleep(1)
        
    final.value = True    
    #p.join()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    
    cflib.crtp.init_drivers()
    

    for link,sequence in zip(uri,sequence):
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            
            reset_estimator(scf)
            # start_position_printing(scf)
            run_sequence(scf, sequence)
    
