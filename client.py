from socket_class import SocketClass


class SocketClient(SocketClass):
    def __init__(self, addr: str, port: int):
        super().__init__(addr, port)

    def start(self):
        self.s.connect((self.addr, self.port))
        print(f'Connected with {(self.addr, self.port)}')

    def test(self):
        self.send("GREETINGS")
        data = self.receive()

        print(f"Получены данные: {data}")

    def test2(self):
        self.send("GREETINGS".encode())
        data = self.receive()

        print(f"Получены данные: {data}")


if __name__ == "__main__":
    client = SocketClient("192.168.2.32", 9998)
    client.start()
    client.test2()
