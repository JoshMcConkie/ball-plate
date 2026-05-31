
import subprocess


def set_auto_exp(setting:bool):
    mode = '3' if setting else '1'
    subprocess.run(["v4l2-ctl", "-d", 
                "/dev/video0", 
                f"--set-ctrl=auto_exposure={mode}"])
    
def disable_auto_exp():
    subprocess.run(["v4l2-ctl", "-d", 
                "/dev/video0", 
                "--set-ctrl=auto_exposure=1"])


def set_exposure_linux(level: int):
    subprocess.run(["v4l2-ctl", "-d", 
                "/dev/video0", 
                f"--set-ctrl=brightness={level}"],
                check=True)
    
def get_exposure_linux():
    result =  subprocess.run(
            ["v4l2-ctl", "-d", "/dev/video0", "--get-ctrl=brightness"],
            capture_output=True,
            text=True,
            check=True,
        )
    level = int(result.stdout.split(":")[1].strip())
    return level