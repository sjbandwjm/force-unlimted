from google.protobuf import descriptor_pb2
from google.protobuf.descriptor import FileDescriptor
from typing import Set

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

def register_schema(writer, message_type) -> int:
    """
    Registers the schema for the given protobuf message type with the MCAP writer.
    Returns the schema ID assigned by the writer.
    """
    desc = message_type.DESCRIPTOR
    fds = collect_schema_with_deps(desc.file)
    schema_bytes = fds.SerializeToString()
    schema_name = desc.full_name

    schema_id = writer.register_schema(
        name=schema_name,
        encoding="protobuf",
        data=schema_bytes,
    )
    return schema_id