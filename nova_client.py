"""Nova Dashboard TCP client (port 29999).

Message format (ASCII text):
  send:     Command(Param1,Param2,...)
  receive:  ErrorID,{value1,...,valueN},Command(Param1,Param2,...);
"""

import re
import socket

_ACK_PATTERN = re.compile(r"^(-?\d+),(\{[^}]*\}),(.*);$")


class NovaTcpError(Exception):
    pass


class NovaClient:
    def __init__(self, host="192.168.5.1", port=29999, timeout=5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock = None

    @property
    def connected(self):
        return self._sock is not None

    def connect(self):
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        self._sock = sock

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def send(self, command):
        if self._sock is None:
            raise NovaTcpError("not connected")
        payload = command.strip() + "\n"
        self._sock.sendall(payload.encode("ascii"))
        response = self._read_until(";")
        error_id, values = parse_response(response)
        return error_id, values, response

    def _read_until(self, terminator):
        data = b""
        while terminator not in data.decode("ascii", "ignore"):
            chunk = self._sock.recv(4096)
            if not chunk:
                raise NovaTcpError("connection closed by peer")
            data += chunk
        return data.decode("ascii", "ignore")


def parse_response(response):
    """Parse 'ErrorID,{v1,v2,...},Command(...);' into (error_id, values)."""
    match = _ACK_PATTERN.match(response.strip())
    if match is None:
        raise NovaTcpError("unexpected response: {!r}".format(response))
    error_id = int(match.group(1))
    raw_values = match.group(2)[1:-1]
    values = [v.strip() for v in raw_values.split(",") if v.strip() != ""]
    return error_id, values
