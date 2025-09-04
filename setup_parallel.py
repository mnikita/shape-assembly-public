import argparse
import paramiko
import sys
import yaml
import time
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed


# --------------------------------------------------------- #
# NOTE: Parallel execution not working!
# Use script only for parallel installation or pulling, e.g.
# - python3 setup_parallel.py 32 33 34 -p [-i]
# --------------------------------------------------------- #

GITHUB_USER = 'TODO'
GITHUB_TOKEN = 'TODO'
GITHUB_EMAIL = 'TODO' # used for commits (can be any)

def ssh_connect(hostname, username='pi', password='raspberry'):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(hostname, username=username, password=password)
        return client
    except Exception as e:
        print(f"SSH connection failed to {hostname}: {e}")
        return None

def run_command(ssh, command, printout=False):
    try:
        stdin, stdout, stderr = ssh.exec_command(command)
        output = stdout.read().decode()
        error = stderr.read().decode()
        if printout:
            print(f"Execution output:", output.strip())
        if error and not (error.strip().startswith("From https://github.com") or error.strip().startswith("Cloning into")):
            print(f"Error in command '{command}': {error}")
        return output.strip()
    except Exception as e:
        output = f"Failed to run command '{command}': {e}"
        return output

def run_command_realtime(ssh, command, timeout=None, robot_id=None):
    """Improved real-time command execution with better process management"""
    try:
        robot_prefix = f"[pi-puck{robot_id}] " if robot_id else ""
        print(f"{robot_prefix}Executing: {command}")
        
        # Use get_pty=True for better process control
        stdin, stdout, stderr = ssh.exec_command(command, get_pty=True)
        
        if timeout:
            stdout.channel.settimeout(timeout)
            stderr.channel.settimeout(timeout)
        
        # Start time for timeout tracking
        start_time = time.time()
        
        while True:
            # Check timeout
            if timeout and (time.time() - start_time) > timeout:
                print(f"{robot_prefix}Command timed out after {timeout} seconds")
                stdout.channel.close()
                return False
            
            # Check if process is still running
            if stdout.channel.exit_status_ready():
                break
                
            # Read stdout
            if stdout.channel.recv_ready():
                try:
                    output = stdout.channel.recv(1024).decode('utf-8', errors='ignore')
                    if output:
                        print(f"{robot_prefix}{output}", end='', flush=True)
                except:
                    pass
            
            # Read stderr
            if stderr.channel.recv_stderr_ready():
                try:
                    error = stderr.channel.recv_stderr(1024).decode('utf-8', errors='ignore')
                    if error:
                        print(f"{robot_prefix}ERROR: {error}", end='', flush=True)
                except:
                    pass
            
            time.sleep(0.1) # delay to prevent busy waiting
        
        exit_status = stdout.channel.recv_exit_status()
        
        # Read any remaining output
        try:
            remaining_output = stdout.read().decode('utf-8', errors='ignore')
            if remaining_output:
                print(f"{robot_prefix}{remaining_output}", end='', flush=True)
        except:
            pass
        
        try:
            remaining_error = stderr.read().decode('utf-8', errors='ignore')
            if remaining_error:
                print(f"{robot_prefix}FINAL ERROR: {remaining_error}", end='', flush=True)
        except:
            pass
        
        print(f"{robot_prefix}Command completed with exit status: {exit_status}")
        return exit_status == 0
        
    except Exception as e:
        print(f"{robot_prefix}Failed to run command '{command}': {e}")
        return False

def run_command_background(ssh, command, robot_id=None):
    """Run command in background and return process ID"""
    try:
        robot_prefix = f"[pi-puck{robot_id}] " if robot_id else ""
        print(f"{robot_prefix}Starting background command: {command}")
        
        # Use nohup to keep process running after SSH disconnection
        bg_command = f"nohup {command} > /tmp/controller_{robot_id}.log 2>&1 & echo $!"
        stdin, stdout, stderr = ssh.exec_command(bg_command)
        
        pid = stdout.read().decode().strip()
        if pid and pid.isdigit():
            print(f"{robot_prefix}Controller started with PID: {pid}")
            return pid
        else:
            print(f"{robot_prefix}Failed to get PID")
            return None
            
    except Exception as e:
        print(f"{robot_prefix}Failed to start background command: {e}")
        return None

def stop_controller(ssh, robot_id=None):
    """Stop the running controller on a robot"""
    try:
        robot_prefix = f"[pi-puck{robot_id}] " if robot_id else ""
        print(f"{robot_prefix}Stopping controller...")
        
        # Kill any running assemble_shape.py processes
        run_command(ssh, "pkill -f assemble_shape.py")
        print(f"{robot_prefix}Controller stopped")
        return True
        
    except Exception as e:
        print(f"{robot_prefix}Failed to stop controller: {e}")
        return False

def install_pipuck(ssh, robot_id=None):    # if it fails, try: sudo rm -r pi-puck
    robot_prefix = f"[pi-puck{robot_id}] " if robot_id else ""
    print(f"{robot_prefix}Installing pi-puck...")
    
    # Clone pi-puck if not exists
    run_command(ssh, "test -d pi-puck || git clone https://github.com/genkimiyauchi/pi-puck.git")
    run_command(ssh, "python3 -m pip install ./pi-puck/python-library VL53L1X")
    print(f"{robot_prefix}pi-puck installation completed")

def setup_repo(ssh, robot_id=None):
    robot_prefix = f"[pi-puck{robot_id}] " if robot_id else ""
    print(f"{robot_prefix}Setting up repository...")
    
    # Clone shape-assembly if not exists
    clone_cmd = f"git clone https://{GITHUB_USER}:{GITHUB_TOKEN}@github.com/{GITHUB_USER}/shape-assembly.git"
    run_command(ssh, f"test -d shape-assembly || {clone_cmd}")
    run_command(ssh, f"cd shape-assembly && git config user.name {GITHUB_USER}")
    run_command(ssh, f"cd shape-assembly && git config user.email {GITHUB_EMAIL}")
    run_command(ssh, f"cd shape-assembly && git stash --include-untracked")  # stash everything just in case
    run_command(ssh, "cd shape-assembly && "
            "git reset --hard HEAD && "         # reset all changes
            "git clean -fd")                    # remove untracked files and folders
    run_command(ssh, f"cd shape-assembly && git pull")
    print(f"{robot_prefix}Repository setup completed")

def re_install_repo(ssh, robot_id=None):
    robot_prefix = f"[pi-puck{robot_id}] " if robot_id else ""
    print(f"{robot_prefix}Reinstalling repository...")
    
    # Clone shape-assembly if not exists
    clone_cmd = f"git clone https://{GITHUB_USER}:{GITHUB_TOKEN}@github.com/{GITHUB_USER}/shape-assembly.git"
    run_command(ssh, f"(test -d shape-assembly && sudo rm -r shape-assembly); {clone_cmd}")
    print(f"{robot_prefix}Repository reinstallation completed")

def install_requirements(ssh, robot_id=None):
    robot_prefix = f"[pi-puck{robot_id}] " if robot_id else ""
    print(f"{robot_prefix}Installing requirements...")
    
    # fixes installation issue for numpy 1.16.2
    run_command(ssh, "sudo DEBIAN_FRONTEND=noninteractive apt-get update -y && "
                     "sudo DEBIAN_FRONTEND=noninteractive apt-get install -y libatlas-base-dev")
    
    run_command(ssh, "cd shape-assembly && python3 -m pip install -r pipuck_requirements.txt")
    print(f"{robot_prefix}Requirements installation completed")

def update_config(ssh, my_id):
    # if permissions needed add: chmod u+w /home/pi/shape-assembly/config_real.yaml
    config_path = "/home/pi/shape-assembly/config_real.yaml"
    try:
        # Download the file
        sftp = ssh.open_sftp()
        with sftp.open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        config['my_id'] = str(my_id)
        if 'ids' not in config or not isinstance(config['ids'], list):
            config['ids'] = [str(my_id)]
        if str(my_id) not in config['ids']:
            config['ids'].append(str(my_id))

        # Upload the updated file
        with sftp.open(config_path, 'w') as f:
            yaml.dump(config, f)
        sftp.close()
        print(f"Successfully updated config_real.yaml with my id: {my_id}.")
    except Exception as e:
        print(f"Error updating config_real.yaml: {e}")

def execute_controller(ssh, robot_id=None):
    """Execute controller"""
    robot_prefix = f"[pi-puck{robot_id}] " if robot_id else ""
    
    # Debug: show current config
    config_path = "/home/pi/shape-assembly/config_real.yaml"
    try:
        sftp = ssh.open_sftp()
        with sftp.open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            print(f"{robot_prefix}MY_ID: {config['my_id']}")
    except Exception as e:
        print(f"{robot_prefix}Could not read config: {e}")

    command = (
        "cd shape-assembly && "
        "python3 ./webots/controllers/assemble_shape/assemble_shape.py"
    )
    
    try:
        # Use background execution
        pid = run_command_background(ssh, command, robot_id)
        if pid:
            print(f"{robot_prefix}Controller started successfully in background")
            print(f"{robot_prefix}To monitor logs: tail -f /tmp/controller_{robot_id}.log")
            print(f"{robot_prefix}To stop: pkill -f assemble_shape.py")
            return True
        else:
            print(f"{robot_prefix}Failed to start controller")
            return False
            
    except KeyboardInterrupt:
        print(f"\n{robot_prefix}Execution interrupted by user.")
        stop_controller(ssh, robot_id)
        return False

def process_robot(robot_id, args):
    """Process a single robot - setup and optionally execute"""
    hostname = f"pi-puck{robot_id}"
    print(f"\n{'='*50}")
    print(f"Processing {hostname}")
    print(f"{'='*50}")
    
    ssh = ssh_connect(hostname)
    if not ssh:
        print(f"Failed to connect to {hostname}, skipping...")
        return False
    
    try:
        if args.execute:
            # Only execute controller
            execute_controller(ssh, robot_id)
            return True
        
        # Full setup process
        install_pipuck(ssh, robot_id)
        
        if args.install:
            re_install_repo(ssh, robot_id)
            setup_repo(ssh, robot_id)
            install_requirements(ssh, robot_id)
        else:
            setup_repo(ssh, robot_id)
        
        update_config(ssh, robot_id)
        
        if args.auto_execute:
            execute_controller(ssh, robot_id)
        else:
            confirm = input(f"Execute controller on {hostname}? (y/n): ")
            if confirm.lower() == 'y':
                execute_controller(ssh, robot_id)
        
        return True
        
    except Exception as e:
        print(f"Error processing {hostname}: {e}")
        return False
    finally:
        ssh.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('ids', nargs='+', help="IDs of the pi-pucks (e.g., 32 33 34)")
    parser.add_argument('--install', '-i', action='store_true', 
                        help="Reinstall repo and update dependencies from pipuck_requirements.txt")
    parser.add_argument('--execute', '-e', action='store_true', 
                        help="Only execute controller script")
    parser.add_argument('--auto-execute', '-a', action='store_true',
                        help="Automatically execute controller after setup")
    parser.add_argument('--parallel', '-p', action='store_true', 
                        help="Process robots in parallel (no debug)")
    args = parser.parse_args()

    # Validate robot IDs
    robot_ids = []
    for id_str in args.ids:
        try:
            robot_id = int(id_str)
            robot_ids.append(robot_id)
        except ValueError:
            print(f"Invalid robot ID: {id_str}. Must be a number.")
            sys.exit(1)
    
    print(f"Processing {len(robot_ids)} robots: {robot_ids}")
    
    if args.parallel and len(robot_ids) > 1:
        # Process robots in parallel
        print("Processing robots in parallel...")
        with ThreadPoolExecutor(max_workers=len(robot_ids)) as executor:
            futures = {executor.submit(process_robot, robot_id, args): robot_id for robot_id in robot_ids}
            
            for future in as_completed(futures):
                robot_id = futures[future]
                try:
                    success = future.result()
                    status = "SUCCESS" if success else "FAILED"
                    print(f"Robot {robot_id}: {status}")
                except Exception as e:
                    print(f"Robot {robot_id}: ERROR - {e}")
    else:
        # Process robots sequentially
        for robot_id in robot_ids:
            process_robot(robot_id, args)

if __name__ == "__main__":
    main()
