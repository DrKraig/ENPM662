#!/usr/bin/env python3

import argparse
from sys import exit, stdout
import subprocess as sp

parser = argparse.ArgumentParser(
    description=
    "Create and run docker container for ShadowHand and ZED Camera. They will be named ShadowContainer and ZedContainer respectively."
)
parser.add_argument("--with_zed",
                    action="store_true",
                    help="Launch ZED camera ROS wrapper?")
parser.add_argument("--nuc", action="store_true", help="Launch the NUC (headless) version of the dockerfile, without GUI features")
parser.add_argument("--host_dir",
                    type=str,
                    default="~/Documents/Projects/ShadowProject/src",
                    help="Project directory on host PC")
parser.add_argument("--container_dir",
                    type=str,
                    default="/home/user/workspace/src",
                    help="Project directory inside container")
parser.add_argument(
    "--other_dirs",
    type=str,
    nargs='+',
    help="Map other directories from host to container, eg: /host/:/container")
parser.add_argument("--local_uid",
                    type=str,
                    default="1000",
                    help="Get local user ID using id -u (default= 1000)")
parser.add_argument("--display",
                    type=str,
                    default=":0",
                    help="Get DISPLAY using echo \$DISPLAY (default= :0)")
parser.add_argument("--nvidia",
                    action="store_true",
                    help="Use nvidia drivers, otherwise default runc")
parser.add_argument("--devices",
                    action="store_true",
                    help="Enable to allow USB devices to be accessible.")
parser.add_argument("--check",
                    action="store_true",
                    help="Show command without executing.")
parser.add_argument("--clean",
                    action="store_true",
                    help="Create a clean container from image")
args = parser.parse_args()

shadow_image_name = "shadow_image"
shadow_container_name = "ShadowContainer"
zed_container_name = "ZedContainer"

if args.devices:
    devices_str = "--device=/dev:/dev"
else:
    devices_str = ""

if args.nvidia:
    runtime_str = "--runtime=nvidia"
    gpu_str = "--gpus all"
else:
    runtime_str = "--runtime=io.containerd.runc.v2"
    gpu_str = ""

if args.other_dirs is not None:
    vol_map = ""
    for each_dir in args.other_dirs:
        vol_map += f"-v {each_dir}:rw "
else:
    vol_map = ""

shadow_docker_str = f"docker run -d -it --name={shadow_container_name} -e interface=eth1 -e DISPLAY={args.display} -e LOCAL_USER_ID={args.local_uid} -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v {args.host_dir}:{args.container_dir}:rw {vol_map} --security-opt seccomp=unconfined --network=host --pid=host {gpu_str} --privileged {devices_str} {runtime_str} {shadow_image_name}:latest"
nuc_docker_str = f"docker run -d -it --name={shadow_container_name} -e interface=eth1 -e LOCAL_USER_ID={args.local_uid} -v {args.host_dir}:{args.container_dir}:rw {vol_map} --security-opt seccomp=unconfined --network=host --pid=host {gpu_str} --privileged {devices_str} {runtime_str} {shadow_image_name}:latest"
zed_docker_str = f"docker run -d -it --name={zed_container_name} --gpus all --privileged --network=host -e DISPLAY={args.display} -v /tmp/.X11-unix:/tmp/.X11-unix:rw zed-image:latest"

if args.check:
    if args.with_zed:
        print(zed_docker_str)
    if args.nuc:
        print(nuc_docker_str)
    else:
        print(shadow_docker_str)
    print() 
    exit(0)
else:
    if args.clean:
        for c in [shadow_container_name, zed_container_name]:
            print(f"Checking if {c} is present...")
            check_container_cmd = f"docker container ls --all | grep -c \"{c}\""
            check_container = int((sp.run(check_container_cmd,
                                          shell=True,
                                          stdout=sp.PIPE)).stdout.decode())
            # Verify that container is actually running
            if check_container != 0:
                print(f"Stopping and removing {c}...")
                stop_container = f"docker stop {c}"
                remove_container = f"docker rm {c}"
                try:
                    res = sp.check_output(stop_container, shell=True)
                    res = sp.check_output(remove_container, shell=True)
                except sp.CalledProcessError as e:
                    print(f"ErrorCode: {e.returncode}")
            else:
                print(f"Container {c} is not present and/or running.")
    try:
        if args.nuc:
            from time import sleep
            sleep(2)
            res = sp.check_output(nuc_docker_str, shell=True)
        else:
            from time import sleep
            sleep(2)
            res = sp.check_output(shadow_docker_str, shell=True)
        if args.with_zed:
            from time import sleep
            sleep(2)
            res = sp.check_output(zed_docker_str, shell=True)
    except sp.CalledProcessError as e:
        print(f"ErrorCode: {e.returncode}")
