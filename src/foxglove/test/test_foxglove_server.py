import time
import random
import struct
from io import BytesIO
import asyncio
import base64
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer
from google.protobuf import descriptor_pb2
from google.protobuf.descriptor import FileDescriptor
from typing import Set

import os, sys
this_file = os.path.abspath(__file__)
project_root = os.path.abspath(os.path.join(os.path.dirname(this_file), '../..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, '../proto/generate'))
from foxglove.Grid_pb2 import Grid
from foxglove.PackedElementField_pb2 import PackedElementField

def collect_schema_with_deps(desc: FileDescriptor) -> descriptor_pb2.FileDescriptorSet:
    """
    加载 proto schema 以及它的所有依赖，返回 FileDescriptorSet
    """
    visited: Set[str] = set()
    fds = descriptor_pb2.FileDescriptorSet()

    def add_deps(fd: FileDescriptor):
        if fd.name in visited:
            return
        visited.add(fd.name)

        # 拷贝当前 proto 描述信息
        proto = descriptor_pb2.FileDescriptorProto()
        fd.CopyToProto(proto)
        fds.file.append(proto)

        # 递归处理依赖
        for dep in fd.dependencies:
            add_deps(dep)

    add_deps(desc)
    return fds


async def main():
    async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
        grid = Grid()
        grid.timestamp.FromSeconds(int(time.time()))
        grid.frame_id = "map"
        grid.pose.position.x = 5.0
        grid.pose.position.y = 0.0
        grid.pose.position.z = 0.0
        grid.pose.orientation.x = 0.0
        grid.pose.orientation.y = 0.0
        grid.pose.orientation.z = 0.0
        grid.pose.orientation.w = 1.0
        grid.column_count = 10
        grid.cell_size.x = 1.0
        grid.cell_size.y = 1.0
        grid.cell_stride = 16  # 4 bytes per float
        grid.row_stride = grid.column_count * grid.cell_stride  # 10 columns * 4 bytes per float
        field = PackedElementField()
        field.name = "red"
        field.offset = 0
        field.type = PackedElementField.FLOAT32
        grid.fields.append(field)
        field.name = "green"
        field.offset = 4
        field.type = PackedElementField.FLOAT32
        grid.fields.append(field)
        field.name = "blue"
        field.offset = 8
        field.type = PackedElementField.FLOAT32
        grid.fields.append(field)
        field.name = "alpha"
        field.offset = 12
        field.type = PackedElementField.FLOAT32
        grid.fields.append(field)
        field = PackedElementField()
        # field.name = "intensity"
        # field.offset = 2
        # field.type = PackedElementField.FLOAT32
        # grid.fields.append(field)
        # Create data: 10x10 grid with intensity values
        data = BytesIO()
        for i in range(grid.column_count - 5):
            for j in range(grid.column_count):
                # intensity = float(1)
                # data.append(struct.pack("<I", i * j))  # Red color
                if j == 0:
                    data.write(
                        struct.pack(
                            "<ffff",
                            0,0,1,
                            0.5
                        )
                    )
                else:
                    data.write(
                        struct.pack(
                            "<ffff",
                            1,0,0,
                            # random.randint(0, 1),
                            # random.randint(0, 1),
                            # random.randint(0, 1),
                            0.5
                        )
                    )
        grid.data = data.getvalue()

        # # Publish the grid message
        fds = collect_schema_with_deps(Grid.DESCRIPTOR.file)
        id = await server.add_channel({
            "topic": "grid",
            "encoding": "protobuf",
            "schemaName": "foxglove.Grid",
            "schema": base64.b64encode(fds.SerializeToString()).decode("utf-8"),
        })
        # print(fds.SerializeToString())
        while True:
            # 发送消息
            await server.send_message(id, time.time_ns(), grid.SerializeToString())
            # print(grid.SerializeToString())
            print("sending")
            # 等待 1 秒
            await asyncio.sleep(1)
        print("🚀 Grid message sent. Server running at ws://localhost:8765")

        # 永久挂起，保持服务器运行
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())


