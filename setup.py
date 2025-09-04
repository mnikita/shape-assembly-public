import argparse
import paramiko
import sys
import yaml
import time

# --------------------------------------------------------- #
# NOTE: Execution might crash after some time running..
# - python3 setup.py <id> [-i]
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
        print(f"SSH connection failed: {e}")
        sys.exit(1)

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

def run_command_realtime(ssh, command, timeout=None):
    try:
        print(f"Executing: {command}")
        stdin, stdout, stderr = ssh.exec_command(command)
        
        if timeout:
            stdout.channel.settimeout(timeout)
            stderr.channel.settimeout(timeout)
        
        while True:
            # check if process is still running
            if stdout.channel.exit_status_ready():
                break
                
            if stdout.channel.recv_ready():
                output = stdout.channel.recv(1024).decode('utf-8', errors='ignore')
                if output:
                    print(output, end='', flush=True)
            
            if stderr.channel.recv_stderr_ready():
                error = stderr.channel.recv_stderr(1024).decode('utf-8', errors='ignore')
                if error:
                    print(f"ERROR: {error}", end='', flush=True)
            
            time.sleep(0.1) # delay to prevent busy waiting
        
        exit_status = stdout.channel.recv_exit_status()
        
        remaining_output = stdout.read().decode('utf-8', errors='ignore')
        if remaining_output:
            print(remaining_output, end='', flush=True)
        
        remaining_error = stderr.read().decode('utf-8', errors='ignore')
        if remaining_error:
            print(f"FINAL ERROR: {remaining_error}", end='', flush=True)
        
        print(f"\nCommand completed with exit status: {exit_status}")
        return exit_status == 0
        
    except Exception as e:
        print(f"Failed to run command '{command}': {e}")
        return False

def install_pipuck(ssh):    # if it fails, try: sudo rm -r pi-puck
    # Clone pi-puck if not exists
    run_command(ssh, "test -d pi-puck || git clone https://github.com/genkimiyauchi/pi-puck.git")
    run_command(ssh, "python3 -m pip install ./pi-puck/python-library VL53L1X")

def setup_repo(ssh):
    # Clone shape-assembly if not exists
    clone_cmd = f"git clone https://{GITHUB_USER}:{GITHUB_TOKEN}@github.com/{GITHUB_USER}/shape-assembly.git"
    run_command(ssh, f"test -d shape-assembly || {clone_cmd}")
    run_command(ssh, f"cd shape-assembly && git config user.name {GITHUB_USER}")
    run_command(ssh, f"cd shape-assembly && git config user.email {GITHUB_EMAIL}")
    run_command(ssh, f"cd shape-assembly && git stash --include-untracked")  # stash everything just in case"
    run_command(ssh, "cd shape-assembly && "
                     "git reset --hard HEAD && "         # reset all changes
                     "git clean -fd")                    # remove untracked files and folders
    run_command(ssh, f"cd shape-assembly && git pull")

def re_install_repo(ssh):
    # Clone shape-assembly if not exists
    clone_cmd = f"git clone https://{GITHUB_USER}:{GITHUB_TOKEN}@github.com/{GITHUB_USER}/shape-assembly.git"
    run_command(ssh, f"(test -d shape-assembly && sudo rm -r shape-assembly); {clone_cmd}")

def install_requirements(ssh):
    # fixes installation issue for numpy 1.16.2
    run_command(ssh, "sudo DEBIAN_FRONTEND=noninteractive apt-get update -y && "
                     "sudo DEBIAN_FRONTEND=noninteractive apt-get install -y libatlas-base-dev")
    
    run_command(ssh, "cd shape-assembly && python3 -m pip install -r pipuck_requirements.txt")

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

def execute_controller(ssh):
    command = (
        "cd shape-assembly && "
            "python3 ./webots/controllers/assemble_shape/assemble_shape.py"
        )
    try:
        # Use real-time execution for better debugging
        success = run_command_realtime(ssh, command, timeout=300)  # timeout!
        if success:
            print("Controller execution completed successfully.")
        else:
            print("Controller execution failed or crashed.")
    except KeyboardInterrupt:
        print("\nExecution interrupted by user.")
        # Try to gracefully stop the remote process
        try:
            ssh.exec_command("pkill -f assemble_shape.py")
            print("Sent termination signal to remote process.")
        except:
            print("Could not terminate remote process.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('id', help="ID of the pi-puck (e.g., 32)")
    parser.add_argument('--install', '-i', action='store_true', 
                        help="Reinstall repo and update dependencies from pipuck_requirements.txt")
    parser.add_argument('--execute', '-e', action='store_true', 
                        help="Only execute controller script")
    args = parser.parse_args()

    hostname = f"pi-puck{args.id}"
    ssh = ssh_connect(hostname)

    if args.execute:
        execute_controller(ssh)
        return

    install_pipuck(ssh)

    if args.install:
        re_install_repo(ssh)
        setup_repo(ssh)
        install_requirements(ssh)
    else:
        setup_repo(ssh)

    update_config(ssh, args.id)

    confirm = input("Execute controller? (y/n): ")
    if confirm.lower() == 'y':
        execute_controller(ssh)

    ssh.close()

if __name__ == "__main__":
    main()
