"""
Frontier detection and scoring module.

Provides algorithms for finding and evaluating exploration frontiers.
"""
from .detector import FrontierDetector
from .scorer import FrontierScorer

__all__ = ['FrontierDetector', 'FrontierScorer']
