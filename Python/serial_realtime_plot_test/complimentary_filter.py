import numpy as np

class ComplimentaryFilter():
	def __init__(self, alpha=0.02, dt=0.01):
		self.alpha = alpha
		self.dt = dt

		self.roll = 0.0
		self.pitch = 0.0


	def compute_complimentary_filter(self, acc, gyro):
		# Compute roll and pitch 
		self.roll = (1 - self.alpha) * (self.roll + gyro[0]*self.dt) + self.alpha*acc[0] 
		self.pitch = (1 - self.alpha) * (self.pitch + gyro[1]*self.dt) + self.alpha*acc[1] 




	def debug_roll_pitch(self): 
		print("Roll: {} \t Pitch: {}".format(self.roll, self.pitch))