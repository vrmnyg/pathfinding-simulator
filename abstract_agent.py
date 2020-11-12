"""
Abstract agent
"""

import abc

class AbstractAgent(metaclass=abc.ABCMeta):
	@classmethod
	def __subclasshook__(cls, subclass):
		return (hasattr(subclass, 'run') and
				callable(subclass.run) and
				hasattr(subclass, 'observe') and
				callable(subclass.observe) or
				NotImplemented)

	@abc.abstractmethod
	def run(self):
		raise NotImplementedError

	@abc.abstractmethod
	def observe(self):
		raise NotImplementedError
