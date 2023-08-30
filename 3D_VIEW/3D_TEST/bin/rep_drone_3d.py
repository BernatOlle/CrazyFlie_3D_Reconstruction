import multiprocessing as mp
from cflib.crazyflie import Crazyflie
import cflib.crtp
import matplotlib.pyplot as plt
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import time
import math
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
# from take_photo import Take


class Drone:
    def __init__(self, name):
        # URI to the Crazyflie to connect to
        self.uri_link_100 = 'radio://0/100/2M/E7E7E7E7E7'
        self.uri_link_80 = 'radio://0/100/2M/E7E7E7E702'
        # self.camera = Take(name)

        self.uri = uri_helper.uri_from_env(default=self.uri_link_80)
        # Change the sequence according to your setup
        #             x    y    z  YAW
        '''
        self.sequence = [

            (-0.5, 0.0, 1, 0),
            (-0.434, -0.25, 1, 30),
            (-0.25, -0.434, 1, 60),
            (0.0, -0.5, 1, 90),
            (0.25, -0.434, 1, 120),
            (0.434, -0.25, 1, 150),
            (0.5, 0.0, 1, 180),
            (0.434, 0.25, 1, 210),
            (0.25, 0.434, 1, 240),
            (0.0, 0.5, 1, 270),
            (-0.25, 0.434, 1, 300),
            (-0.434, 0.25, 1, 330),
            (-0.5, 0.0, 1, 0),
            (-0.5, 0.0, 1.25, 0),

        ]
        '''
        self.land = [
            (0, 0, 1.5, 0),
        ]

        self.anchors = [

            (-1.1, -1.0, 0.1),
            (-1.1, 1.0, 2.2),
            (1.1, 1.0, 0.2),
            (1.1, -1.0, 2.2),
            (-1.1, -1.0, 2.2),
            (-1.1, 1.0, 0.1),
            (1.1, 1.0, 2.2),
            (1.1, -1.0, 0.1),
        ]

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

    def update_graph(self, x, y, z, final, i):
        while final.value == False:
            ax = self.fig.add_subplot(111, projection='3d')
            # Grafica los puntos como un scatter plot
            x_values = x.value
            y_values = y.value
            z_values = z.value

            anc_x = [p[0] for p in self.anchors]
            anc_y = [p[1] for p in self.anchors]
            anc_z = [p[2] for p in self.anchors]
            sc = ax.scatter(anc_x*100, anc_y*100, anc_z*100, c='blue')

            sc_red = ax.scatter(round(x_values, 2), round(
                y_values, 2), round(z_values, 2), c='red', s=70)

            ax.text(x_values, y_values, z_values, str(x_values*100) +
                    ' '+str(y_values*100)+' ' + str(z_values*100))

            # Configura los labels de los ejes
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            plt.show(block=False)
            plt.pause(0.02)
            plt.clf()

        plt.close()

    def wait_for_position_estimator(self, scf):
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

    def reset_estimator(self, scf):
        global data
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        data = self.TOC(cf)

        self.wait_for_position_estimator(cf)

    def position_callback(self, timestamp, data, logconf):
        x = data['kalman.stateX']
        y = data['kalman.stateY']
        z = data['kalman.stateZ']
        print('pos: ({}, {}, {})'.format(x, y, z))

    def start_position_printing(self, scf):
        log_conf = LogConfig(name='Position', period_in_ms=500)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.position_callback)
        log_conf.start()

    def has_reached_position(self, position, where):

        print("Recalculating " + where, position[0], position[1], position[2])
        current_position = [data.x, data.y, data.z]
        threshold_distance = 0.15

        distance = ((current_position[0] - position[0]) ** 2 +
                    (current_position[1] - position[1]) ** 2 +
                    (current_position[2] - position[2]) ** 2) ** 0.5

        return distance <= threshold_distance
    '''
    def go_to(self, cf, position):

        for i in range(400):
            cf.commander.send_setpoint(0.0, 0.0, 0, 60000)
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
            # update_graph(position[x],position[y],position[z])
            print("Drona in position: ", data.x, data.y, data.z)
            self.x.value = data.x
            self.y.value = data.y
            self.z.value = data.z
            time.sleep(0.007)
            self.index_d.value += 1

        while not self.has_reached_position(position):
            time.sleep(0.002)
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
    '''
    def sequence_point(self,n_points,distance,z):
        n_points+=1
        theta = np.linspace(0, 2*np.pi, n_points, endpoint=True)
        x = distance * np.cos(theta+np.pi)
        y = distance * np.sin(theta+np.pi)
        z_v = [z]*n_points
        grados = np.degrees(theta)
        self.sequence = list(zip(x, y, z_v, grados))
        return self.sequence
        
    def go_to_2(self, cf, position, pos_ant, where):

        dif = position[2]-pos_ant[2]  # de la 4 a la 2 hi han 2

        k = 20
        for i in range(k):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                pos_ant[2]+((1/(k-1))*i*dif),
                                                position[3])
            # update_graph(position[x],position[y],position[z])
            print("Dron in position " + where+" : ", data.x, data.y, data.z)
            self.x.value = data.x
            self.y.value = data.y
            self.z.value = data.z
            time.sleep(0.1)
            self.index_d.value += 1
        '''
        while not self.has_reached_position(position, where):
            time.sleep(0.1)
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
        '''
    def go_to(self, cf, position, pos_ant, where):

        dif = [position[0]-pos_ant[0], position[1]-pos_ant[1],
               position[2]-pos_ant[2]]  # de la 4 a la 2 hi han 2
        k = 20
        for i in range(k):
            cf.commander.send_position_setpoint(position[0]+((1/(k-1))*i*dif[0]),
                                                position[1] +
                                                ((1/(k-1))*i*dif[1]),
                                                pos_ant[2] +
                                                ((1/(k-1))*i*dif[2]),
                                                position[3])
            # update_graph(position[x],position[y],position[z])
            print("Dron in position "+where+" : ", data.x, data.y, data.z)
            self.x.value = data.x
            self.y.value = data.y
            self.z.value = data.z
            time.sleep(0.1)
            self.index_d.value += 1

        for i in range(20):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                pos_ant[2],
                                                position[3])
            # update_graph(position[x],position[y],position[z])
            print("Drona in position: ", data.x, data.y, data.z)
            self.x.value = data.x
            self.y.value = data.y
            self.z.value = data.z
            time.sleep(0.1)
            self.index_d.value += 1
        
        while not self.has_reached_position(position, where):
            time.sleep(0.2)
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
        
    def go_land(self, cf, initial_val, last_pos):
        pos_ant_vec = [last_pos] + self.land

        self.land.insert(0, last_pos)
        for i, lan in enumerate(self.land):
            print('Setting position {}'.format(lan))
            self.go_to_2(cf, lan, pos_ant_vec[i], "Landing")
            self.index_d.value = 0
            print("NextPoint Landing")
            pos_ant = [lan]

        initial_val = (initial_val[0], initial_val[1], initial_val[2], 0)

        for i in range(6):
            initial_lan = (initial_val[0], initial_val[1],
                           initial_val[2]+(1-(i*(1/5))), 0)
            pos_ant.append(initial_lan)
            self.go_to_2(cf, initial_lan, pos_ant[i], "Landing")

        self.final.value = True
        cf.commander.send_stop_setpoint()
        time.sleep(0.1)

    def initial_run(self, scf):
        self.initial_values = (data.x, data.y, data.z, 0)
        self.cf = scf.cf
        self.cf.commander.send_velocity_world_setpoint(0.05, 0.05, 1, 5)
        self.limit = 0.01
        print(data.x, data.y, data.z)
        self.x = mp.Value('f', 0)
        self.y = mp.Value('f', 0)
        self.z = mp.Value('f', 0)
        self.index_d = mp.Value('i', 0)
        self.final = mp.Value('b', False)
        self.fig = plt.figure(figsize=(15, 15))

        p = mp.Process(target=self.update_graph, args=(
            self.x, self.y, self.z, self.final, self.index_d,))
        p.start()

    def next_point(self, cf, sequence, point):
        if (len(sequence) > point):
            position = sequence[point]
            print('Setting position {}'.format(position))
            self.go_to(cf, position)
            self.index_d.value = 0
            print("NextPoint")
            time.sleep(1)
            return True
        else:
            return False

    def take_off(self, cf, initial_val):
        initial_val = [(initial_val[0], initial_val[1], initial_val[2], 0)]
        print(initial_val)
        for i in range(5):
            
            initial_lan = (initial_val[0][0], initial_val[0]
                           [1], initial_val[0][2]+((i+1)*(1/6)), 0)
            print("Next Take off point", initial_lan)
            initial_val.append(initial_lan)
            self.go_to_2(cf, initial_lan, initial_val[i], "Taking off")

        return initial_lan

    def run_sequence(self, scf, n_points,radius, altitude):
        sequence = self.sequence_point(n_points,radius,altitude)
        initial_values = (data.x, data.y, data.z, 0)
        cf = scf.cf
        cf.commander.send_velocity_world_setpoint(0.05, 0.05, 0.05, 5)
        limit = 0.01
        print(data.x, data.y, data.z)

        self.x = mp.Value('f', 0)
        self.y = mp.Value('f', 0)
        self.z = mp.Value('f', 0)
        self.index_d = mp.Value('i', 0)
        self.final = mp.Value('b', False)
        self.fig = plt.figure(figsize=(15, 15))

        cf.commander.send_setpoint(0.0, 0.0, 0, 0)
        p = mp.Process(target=self.update_graph, args=(
            self.x, self.y, self.z, self.final, self.index_d,))
        p.start()

        last_pos = self.take_off(cf, initial_values)

        for position in sequence:
            print('Setting position {}'.format(position))
            self.go_to(cf, position, last_pos, "Sequence")
            self.index_d.value = 0
            print("NextPoint")
            # self.camera.take_photo()
            last_pos = position

        self.go_land(cf, initial_values, last_pos)
        self.final.value = True
        p.join()

        time.sleep(0.1)

    def ini_drivers(self):
        cflib.crtp.init_drivers()

    def conn_drone(self):
        self.ini_drivers()
        self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache'))
        self.scf.open_link()
        self.reset_estimator(self.scf)

    def run(self, n_points,radius,altitude):
        cflib.crtp.init_drivers()

        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:

            self.reset_estimator(scf)
            # start_position_printing(scf)
            self.run_sequence(scf, n_points,radius,altitude)


if __name__ == '__main__':
    drone = Drone("Bernat")
    drone.run(12,0.5,1)
