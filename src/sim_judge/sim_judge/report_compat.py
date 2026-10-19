"""RoboCommand report.proto bindings that load on any protobuf runtime >= 3.20.

report_pb2.py is RoboNation's generated file (protobuf 6.32, edition 2023), vendored here so the
judge decodes with the official schema independently of the team's message_node. Older runtimes
(Ubuntu 24.04 ships protobuf 4.21) cannot import it, so the classes are rebuilt from its embedded
descriptor declared as proto2, which has the same wire format and explicit field presence.
"""

try:
    from .report_pb2 import *  # noqa: F401,F403
except ImportError:
    import ast
    import os
    import re

    from google.protobuf import descriptor_pb2
    from google.protobuf import descriptor_pool
    from google.protobuf import timestamp_pb2  # noqa: F401  (registers the Timestamp dependency)
    from google.protobuf.internal import builder as _builder

    with open(os.path.join(os.path.dirname(__file__), 'report_pb2.py'), encoding='utf-8') as _f:
        _match = re.search(r"AddSerializedFile\((b'(?:[^'\\]|\\.)*')\)", _f.read())
    _file = descriptor_pb2.FileDescriptorProto.FromString(ast.literal_eval(_match.group(1)))
    if _file.syntax == 'editions':
        _file.syntax = 'proto2'
        _file.DiscardUnknownFields()
    # A private pool keeps this copy from clashing with message_node's when both load in one process.
    _POOL = descriptor_pool.DescriptorPool()
    _POOL.AddSerializedFile(timestamp_pb2.DESCRIPTOR.serialized_pb)
    DESCRIPTOR = _POOL.AddSerializedFile(_file.SerializeToString())
    _builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
    _builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'sim_judge.report_pb2', globals())
