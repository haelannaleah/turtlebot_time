from logger import Logger

class TurtlebotClass():
    def __init__(self, name):
        self.__name__ = name
        self._logger = Logger(self.__name__)