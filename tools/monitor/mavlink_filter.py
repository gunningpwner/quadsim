import sys
import os
from platformio.public import DeviceMonitorFilterBase

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
        self.mav = mavlink_dialect.MAVLink(file=open(os.devnull, 'wb'))
        print("--- MAVLink filter started ---")

    def __call__(self):
        return self

    def rx(self, text):
        """Called with received data."""
        # The `text` parameter is a bytes object. We feed it byte by byte to the parser.
        msgs = self.mav.parse_buffer(text)
        if msgs:
            for msg in msgs:
                self.print_message(msg)
        return b'' # Don't print the raw data

    def tx(self, text):
        """Called with transmitted data."""
        return text

    def print_message(self, msg):
        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            # MAVLink STATUSTEXT messages are not null-terminated in the payload.
            # We must handle this to print them cleanly.
            text = msg.text.decode(sys.getdefaultencoding(), 'ignore').rstrip('\0')
            self.writer.println(f"STATUSTEXT: {text}")
        elif msg_type == "ATTITUDE_QUATERNION":
            self.writer.println(f"ATTITUDE_QUATERNION: w={msg.q1:.3f}, x={msg.q2:.3f}, y={msg.q3:.3f}, z={msg.q4:.3f}")
        else:
            # Print any other message type in a generic format
            self.writer.println(str(msg))
        self.writer.flush()