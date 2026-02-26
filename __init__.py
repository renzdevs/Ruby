"""
hardware/
FlexBone-X locomotion controller + PurrSynth audio interface.
"""
from .locomotion import LocomotionController
from .audio import PurrSynth

__all__ = ["LocomotionController", "PurrSynth"]
