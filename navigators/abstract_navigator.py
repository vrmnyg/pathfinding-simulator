"""
Abstract navigator
"""

import abc
from abc import ABC, abstractmethod

class AbstractNavigator(metaclass=abc.ABCMeta):
	@classmethod
	def __subclasshook__(cls, subclass):
		return (hasattr(subclass, 'run') and
				callable(subclass.run)
				or
				NotImplemented)

	@abstractmethod
	def run(self):
		raise NotImplementedError
