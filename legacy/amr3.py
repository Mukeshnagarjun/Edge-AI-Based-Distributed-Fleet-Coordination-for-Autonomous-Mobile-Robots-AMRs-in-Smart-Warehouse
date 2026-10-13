import subprocess
import sys
from pathlib import Path

subprocess.run([sys.executable, str(Path(__file__).with_name("p2p_agent.py")), "AMR-3"])
