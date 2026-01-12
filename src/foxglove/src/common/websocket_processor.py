from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer

from .processor_base import FoxgloveProcessor
from .message import OutMessage

class WebsocketProcessor(FoxgloveProcessor):
    """WebSocket数据处理类"""
    def Init(self, args):
        self._server = FoxgloveServer(args.ip, args.port, "Foxglove WebSocket Server")
        run_cancellable(self._server.start())
        pass

    def Process(self, msg: OutMessage):
        pass

# async def main():
#     parse = argparse.ArgumentParser()
#     args = parse.parse_args()
#     async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
#         # # Publish the grid message
#         fds = collect_schema_with_deps(Grid.DESCRIPTOR.file)
#         id = await server.add_channel({
#             "topic": "grid",
#             "encoding": "protobuf",
#             "schemaName": "foxglove.Grid",
#             "schema": base64.b64encode(fds.SerializeToString()).decode("utf-8"),
#         })
#         # print(fds.SerializeToString())
#         while True:
#             # 发送消息
#             await server.send_message(id, time.time_ns(), Grid().SerializeToString())
#             # print(grid.SerializeToString())
#             print("sending")
#             # 等待 1 秒
#             await asyncio.sleep(1)

#         # 永久挂起，保持服务器运行
#         await asyncio.Future()