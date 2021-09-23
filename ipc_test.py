import os
import time
from subprocess import Popen

client_command = ["ros2", "run", "ipc_test_client", "ipc_test_client"]
service_command = ["ros2", "run", "ipc_test_service", "ipc_test_service"]
clients_cnt = 5
client_procs = [None for i in range(clients_cnt)]
service_procs = [None]
requests_cnt = 0


def terminate_procs(procs):
    for p in procs:
        if p:
            p.terminate()


def terminate_all_procs():
    terminate_procs(service_procs)
    terminate_procs(client_procs)


def create_proc(proc_command, requests_cnt):
    return Popen(proc_command + ["_" + str(requests_cnt)])


def create_service_proc(proc_command):
    return Popen(proc_command)


def create_client_proc(proc_command):
    global requests_cnt
    requests_cnt += 1
    return create_proc(proc_command, requests_cnt)


def handle_proc(procs, proc_name, proc_command, create_proc_fn):
    for i in range(len(procs)):
        if procs[i] is None:
            procs[i] = create_proc_fn(proc_command)
        else:
            ret = procs[i].poll()
            if ret is not None:
                if ret != 0:
                    terminate_all_procs()
                    raise Exception(proc_name, "terminated with signal", ret)
                procs[i] = None


def handle_service():
    handle_proc(service_procs, "Service", service_command, create_service_proc)


def handle_clients():
    handle_proc(client_procs, "Client", client_command, create_client_proc)


while True:
    handle_service()
    handle_clients()
    time.sleep(5)
