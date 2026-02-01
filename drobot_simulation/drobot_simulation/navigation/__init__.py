"""
Navigation module for collision handling and recovery behaviors.

Provides collision avoidance and recovery maneuvers for exploration.
"""
from .collision import CollisionHandler
from .recovery import RecoveryManager

__all__ = ['CollisionHandler', 'RecoveryManager']
