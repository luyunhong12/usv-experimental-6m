"""One provided actuator-command source per boat and Gazebo partition."""
import fcntl,hashlib,os,tempfile
from pathlib import Path
class ControlLease:
    def __init__(self,boat,owner):
        partition=os.environ.get('GZ_PARTITION','default')
        token=hashlib.sha256((str(os.getuid())+'|'+partition+'|'+boat).encode()).hexdigest()[:24]
        self.path=Path(tempfile.gettempdir())/('usv-control-'+token+'.lock')
        self.file=self.path.open('a+')
        try:fcntl.flock(self.file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            self.file.seek(0);other=self.file.read().strip();self.file.close()
            raise RuntimeError('Actuator controller already active: '+other+'. Stop it or launch interfaces with control:=False.')
        self.file.seek(0);self.file.truncate();self.file.write(f'{owner} pid={os.getpid()} boat={boat}');self.file.flush()
    def close(self):
        if not self.file.closed:fcntl.flock(self.file,fcntl.LOCK_UN);self.file.close()
