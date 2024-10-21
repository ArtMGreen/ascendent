import socket
from typing import List


class SocketClass:
    addr: str
    port: int
    s: socket.socket

    def __init__(self, addr: str, port: int):
        self.addr = addr
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    @staticmethod
    def encode(data: str | bytes) -> bytes:
        """
        header=CASINO
        ?
        type=str|bytes
        ?
        data=str
        """
        return f"CASINO?{type(data).__name__}?{data}".encode()

    @staticmethod
    def decode(msg: bytes) -> List[str]:
        """
        header=CASINO
        ?
        type=str|bytes
        ?
        data=str
        """
        return msg.decode().split('?')

    def receive(self, conn: None | socket.socket = None) -> str:
        if conn is None:
            conn = self.s

        msg = conn.recv(1024)

        if not msg:
            return ""

        header, type_, data = self.decode(msg)

        if not (header == 'CASINO'):
            raise Exception(f'Wrong Header')

        if not (type_ in ('str', 'bytes')):
            raise Exception(f'Wrong Type. Expected str or bytes, got {type_}')

        return data

    def send(self, msg: str | bytes, conn: None | socket.socket = None) -> None:
        if conn is None:
            conn = self.s

        if not msg:
            return

        conn.send(self.encode(msg))
