from abc import ABC, abstractmethod
import argparse as args

from .message import OutMessage

class FoxgloveProcessor(ABC):
    """数据处理的模板类"""
    @abstractmethod
    def Init(self, args: args.Namespace):
        pass

    @abstractmethod
    def Process(self, msg: OutMessage):
        pass

