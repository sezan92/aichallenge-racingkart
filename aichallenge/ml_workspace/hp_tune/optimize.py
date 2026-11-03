import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import optuna
import yaml

AICHALLENGE_DIR = Path("/aichallenge")
WORKSPACE_DIR = AICHALLENGE_DIR / "workspace"

def suggest_mpc_params(trial: optuna.Trial) -> Dict:
    return {
        # TO BE ADDED
    }
