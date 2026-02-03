from typing import List, Dict, Any, Optional, Tuple
import numpy as np

class DataLogger:
    def __init__(self):
        self.data: Dict[str, List[list]] = {}

    def log(self, time: float, **kwargs):
        """
        Logs variables with their timestamp.
        Usage: logger.log(t, acc=np.array([0,0,9.8]), alt=100.0)
        Structure stored: [[t, x, y, z], [t, x, y, z], ...]
        """
        for key, val in kwargs.items():
            if key not in self.data:
                self.data[key] = []
            
            # Normalize input to a flat list so we can prepend time
            if np.isscalar(val):
                val_list = [val]
            elif isinstance(val, np.ndarray):
                val_list = val.flatten().tolist()
            elif isinstance(val, list):
                val_list = val
            else:
                # Fallback for other iterables
                val_list = list(val)
            
            # Store [Time, Data_0, Data_1, ...]
            self.data[key].append([time] + val_list)

    def get_arrays(self):
        """
        Returns dictionary of numpy arrays.
        Format: key -> NxM array where Column 0 is Time.
        """
        return {k: np.array(v) for k, v in self.data.items()}