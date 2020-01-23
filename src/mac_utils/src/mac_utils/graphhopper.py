#!/usr/bin/env python3

import subprocess
import shlex
import rospkg
from fcntl import fcntl, F_GETFL, F_SETFL
from os import O_NONBLOCK, path, environ
import threading
import rospy
from mac_utils.srv import SetGraphhopperMap, SetGraphhopperMapResponse

GRAPHHOPPER_DEFAULT_PORT = 8989

class GraphhopperProcessHandler(object):
    """
    This class is a graphhopper process wrapper that starts and stops graphhopper services
    depending on the required map
    """

    # paths are relative to package graph
    GRAPHHOPPER_PATH = '/../../third-party/graphhopper'
    GRAPHHOPPER_CMD = 'graphhopper.sh web '

    MAP_PATH = '/data/osm/'

    MAP_EXTENSION = '.osm.pbf'

    MAP_SERVICE_NAME = 'set_graphhopper_map'

    class GraphHopperProcess(object):

        def __init__(self, map_name, port, process):
            self.map_name = map_name
            self.port = port
            self.process = process

    def __init__(self, maps=[], initial_port=GRAPHHOPPER_DEFAULT_PORT):
        """
        :param map: set a map that is loaded during start
        """
        rospack = rospkg.RosPack()

        self._package_path = rospack.get_path("mac_utils")

        self._processes = {}

        self._next_port = initial_port

        self._set_map_lock = threading.Lock()

        rospy.Service(GraphhopperProcessHandler.MAP_SERVICE_NAME, SetGraphhopperMap, self._set_map_callback)
        # prestarting maps if requested
        for map_name in maps:
            try:
                if map_name:
                    self._set_map(map_name)
            except Exception as e:
                rospy.logerr(e)

    def _set_map(self, map_name):
        """
        set new map, function will either return port of running instance or will create a new instance and then return its port
        :param map_name: map name without path and extension
        :return: graphopper server port
        """
        with self._set_map_lock:
            try:
                g_process = self._processes[map_name]
            except KeyError:
                port = self._next_port
                process = self.start_graphhopper_process(map_name, port)
                self._next_port += 1
                g_process = GraphhopperProcessHandler.GraphHopperProcess(map_name=map_name, port=port, process=process)
                rospy.loginfo("Map '%s' is available on port:%d", map_name, port)

                # store process in dict
                self._processes[map_name] = g_process
            return g_process.port

    def start_graphhopper_process(self, map_name, port):
        """
        start a graphhopper process child
        :param map_name:  map name without path and extension
        :return: process handle
        """
        map_file = self._package_path + GraphhopperProcessHandler.MAP_PATH + map_name + GraphhopperProcessHandler.MAP_EXTENSION
        if path.exists(map_file):

            rospy.loginfo('Creating Graphhopper process for map %s', map_file)

            cmd = self._package_path + GraphhopperProcessHandler.GRAPHHOPPER_PATH + r'/' + GraphhopperProcessHandler.GRAPHHOPPER_CMD + map_file

            # set port as environment variable
            env = environ.copy()
            env['JETTY_PORT'] = str(port)
            process = self._start_process(command=cmd,cwd=self._package_path + GraphhopperProcessHandler.GRAPHHOPPER_PATH, env=env)
            return process

        else:
            raise Exception('Mapfile ' + map_file + ' not available')

    def _set_map_callback(self, req):

        port = self._set_map(req.map)

        response = SetGraphhopperMapResponse()

        response.port = port

        return response

    def _start_process(self, command, cwd , env):
        """
        Execute a process command
        :param command: full command string
        :param cwd: working directory
        :param env: environment
        """
        cmd = shlex.split(command)

        p = subprocess.Popen(cmd, cwd=cwd, shell=False, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, universal_newlines=True)

        flags = fcntl(p.stdout, F_GETFL)  # get current p.stdout flags
        fcntl(p.stdout, F_SETFL, flags | O_NONBLOCK)
        # waiting until server is up and running
        while True:
            try:
                res = p.stdout.readline()
                #if res != '': print(res)
            except Exception as e:
                if e.errno == 11:  # corresponds to "Resource temporarily unavailable" and indicates we do not have more data
                    continue
                else:
                    raise e
            if '[main] INFO  org.eclipse.jetty.server.Server - Started' in res:  # this is unfortunately language specific
                p.stdout.close()
                break
        #time.sleep(4.0)
        return p

    def _check_state(self):
        for gp in list(self._processes.values()):
            try:
                ret = gp.process.poll()
                if ret: # check if the process terminated
                    rospy.logerr('Graphhopper has terminated with ret:%d. Trying a restart...', ret)
                    gp.process = self.start_graphhopper_process(map_name=gp.map_name,port=gp.port)

            except Exception as e:
                if e.errno == 11:  # corresponds to "Resource temporarily unavailable" and indicates we do not have more data
                    break
                else:
                    rospy.logerr(e)

    def _stop_processes(self):

        for gp in list(self._processes.values()):
            gp.process.terminate()
            gp.process.wait()

    def __del__(self):
        self._stop_processes()


if __name__ == '__main__':

    rospy.init_node(name='graphhopper', log_level=rospy.INFO)

    start_maps = rospy.get_param('~start_maps', '')
    initial_port = rospy.get_param('~initial_port', GRAPHHOPPER_DEFAULT_PORT)
    start_maps = start_maps.split(',')

    gh = GraphhopperProcessHandler(maps=start_maps, initial_port=initial_port)

    r = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():
        gh._check_state()
        r.sleep()