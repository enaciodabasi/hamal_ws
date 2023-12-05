import sys
import robot_upstart
import subprocess

num_args = len(sys.argv)

master_uri = ""

def get_current_ip():

    ip_subprocess_res = subprocess.run(['hostname', '-I'], stdout=subprocess.PIPE)

    curr_ip_str = ip_subprocess_res.stdout.decode()

    possible_ips = curr_ip_str.split()

    return possible_ips[0]

host_ip = get_current_ip()

if num_args <= 1: 
    master_uri = host_ip 
elif num_args >= 1:
    master_uri = sys.argv[1] 
    

if master_uri == "":
    sys.exit(-1)

startup_job = robot_upstart.Job(
    name="hamal_startup_ros"
)
startup_job.user = "root"
startup_job.rosdistro = "noetic"
startup_job.master_uri = master_uri
startup_job.symlink = True
startup_job.roslaunch_wait = True

startup_job.add(package="hamal_bringup", filename="launch/hamal_bare_startup.launch")

startup_job.install()

