from abc import ABC, abstractmethod


class Listener(ABC):
    pass


class AxisListener(Listener):

    @abstractmethod
    def onAxisMoveAction(self, x, y) -> None:
        pass


class EventListener(Listener):

    @abstractmethod
    def onButtonDown(self) -> None:
        pass

    @abstractmethod
    def onButtonUp(self) -> None:
        pass

    @abstractmethod
    def onButtonPress(self) -> None:
        pass
