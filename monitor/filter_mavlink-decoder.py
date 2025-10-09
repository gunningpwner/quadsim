import sys
import os
from platformio.public import DeviceMonitorFilterBase
import traceback
try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink_dialect
except ImportError:
    sys.stderr.write(
        "ERROR: pymavlink is not installed for the PlatformIO Python environment.\n"
        "Please open a PlatformIO terminal and run: pip install pymavlink\n"
    )
    sys.exit(1)


class MavlinkDecoder(DeviceMonitorFilterBase):
    NAME = "mavlink-decoder"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.mav = mavlink_dialect.MAVLink(file=None)
        print("--- MAVLink filter started ---")

    def __call__(self):
        return self

    def rx(self, text):
        """Called with received raw bytes."""
        # Because self.is_bytes is True, `text` is a `bytes` object.
        output_lines = []
        try:
            msgs = self.mav.parse_buffer(text.encode('latin-1'))
            if msgs:
                for msg in msgs:
                    # Collect the formatted message strings
                    output_lines.append(self.format_message(msg))
        except Exception as e:
            print(traceback.format_exc())
        # Join all collected lines and return them to be printed in the monitor
        return "".join(output_lines)

    def tx(self, text):
        """Called with transmitted data."""
        return text

    def format_message(self, msg):
        """Formats a MAVLink message into a human-readable string."""
        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            # MAVLink STATUSTEXT messages are not null-terminated in the payload.
            text = msg.text.rstrip('\0')
            return f"STATUSTEXT: {text}\n"
        elif msg_type == "ATTITUDE_QUATERNION":
            return f"ATTITUDE_QUATERNION: w={msg.q1:.3f}, x={msg.q2:.3f}, y={msg.q3:.3f}, z={msg.q4:.3f}\n"
        else:
            return str(msg)+"\n"