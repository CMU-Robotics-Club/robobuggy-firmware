import numpy as np
from numpy import linalg
import time
from dataclasses import dataclass
import matplotlib.pyplot as plt
import matplotlib.animation as animation

@dataclass
class GpsPosition:
	x: float
	y: float

@dataclass
class RawAccelerometer:
	x: float
	y: float
	z: float

@dataclass
class RotationVector:
	r: float
	i: float
	j: float
	k: float

with open('log42.txt', 'r') as f:
	data = f.readlines()

def takeprefix(s: str, prefix: str) -> str:
	assert(s.startswith(prefix))
	return s.removeprefix(prefix)

def takefloat(s):
	for idx in range(0, len(s)):
		if not s[idx].isdigit() and s[idx] not in ('-', '.'):
			assert(idx > 0)
			return float(s[:idx]), s[idx:]
	return float(s), ''

def takeint(s):
	for idx in range(0, len(s)):
		if not s[idx].isdigit() and s[idx] not in ('-'):
			assert(idx > 0)
			return int(s[:idx]), s[idx:]
	return int(s), ''

def parse_args(s: str):
	result = dict()
	rest = s
	while rest != '':
		key, rest = rest.split(': ', maxsplit=1)
		assert(len(key) == 1)
		value, rest = takefloat(rest)
		rest = rest.strip()
		result[key] = value
	return result

def parse_xyz(s: str):
	xyz = parse_args(s)
	assert(len(xyz) == 3)
	return (xyz['x'], xyz['y'], xyz['z'])

def parse_rijk(s: str):
	rijk = parse_args(s)
	assert(len(rijk) == 4)
	return (rijk['r'], rijk['i'], rijk['j'], rijk['k'])
	

assert(takefloat('-123.456 abc') == (-123.456, ' abc'))

parsed = []

for line in data[1:]:
	line = line.strip()
	line = takeprefix(line, 't: ')
	timestamp, line = takeint(line)
	line = takeprefix(line, ', ')
	if (n := line.removeprefix('No fix - Num. Satellites: ')) != line:
		num_satellites = takeint(n)
		print(f'Num Satellites: {num_satellites}')
	elif (rest := line.removeprefix('IMU ')) != line:
		kind, args = rest.split(' - ', 1)
		match kind:
			case 'Raw Accelerometer': 
				parsed.append((timestamp, RawAccelerometer(*parse_xyz(args))))
			case 'Rotation Vector':
				parsed.append((timestamp, RotationVector(*parse_rijk(args))))
			case 'Raw Magnetic Field':
				# Ignore for now
				pass
			case _:
				pass
				#print(f'Unknown IMU reading ({kind}) ({args})')
	else:
		rest = takeprefix(line, 'x: ')
		x, rest = takefloat(rest)
		rest = takeprefix(rest, ', y: ')
		y, rest = takefloat(rest)
		rest = takeprefix(rest, ', time: ')
		ts2, rest = takeint(rest)
		assert(rest == '')

		parsed.append((timestamp, GpsPosition(x, y)))
	
for d in parsed:
	print(d)
	
x_center = 4477490.037884
y_center = 589769.727319

times = []
x_data = []
y_data = []

for t, d in parsed:
	if isinstance(d, GpsPosition):
		if abs(d.x - x_center) > 1000 or abs(d.y - y_center) > 1000:
			print(f'Invalid position {d.x} {d.y}')
			continue

		times.append(t)
		x_data.append(d.x - x_center)
		y_data.append(d.y - y_center)


#plt.ion()
	
fig = plt.figure()
ax = fig.add_subplot()

#scatter = ax.scatter(x_data, y_data)

@dataclass
class KalmanFilter:
	state: np.ndarray
	'''Column vector of [x, y, theta, v_x, v_y]'''

	state_cov: np.ndarray
	'''Matrix of [x, y, theta, v_x, v_y], should be a diagonal matrix'''

	state_transition_mat: np.ndarray
	'''Matrix that updates a state across a fixed timestep'''

	observation_mat: np.ndarray
	'''Matrix that yields expected sensor readings from a given state'''

	process_noise_cov: np.ndarray
	'''Diagonal matrix representing variance of the process noise'''

	observation_noise_cov: np.ndarray

	def prediction_step(self, control_input):
		'''Computes a state estimation at time k + 1 based only on the information at time k and prior.'''

		self.state = self.state_transition_mat @ self.state + control_input

		self.state_cov = self.state_transition_mat @ self.state_cov @ self.state_transition_mat.transpose() + self.process_noise_cov
	
	def update_step(self, sensors: np.ndarray):
		'''Computes a state estimation at time k + 1 based on the raw state estimation at k + 1, combined with sensor readings'''

		innovation = sensors - self.observation_mat @ self.state

		print('Innovation:')
		print(innovation)

		innovation_cov = self.observation_mat @ self.state_cov @ self.observation_mat.transpose() + self.observation_noise_cov

		print('Innovation Covariance:')
		print(innovation_cov)

		print('State Covariance:')
		print(self.state_cov)
		
		kalman_gain = self.state_cov @ self.observation_mat.transpose() @ linalg.inv(innovation_cov)

		print('Gain:')
		print(kalman_gain)

		self.state = self.state + kalman_gain @ innovation

		coeff = kalman_gain @ self.observation_mat
		coeff = np.identity(len(coeff)) - coeff

		self.state_cov = coeff @ self.state_cov @ coeff.transpose() + kalman_gain @ self.observation_noise_cov @ kalman_gain.transpose()

dt = 0.1
	
kalman_filter = KalmanFilter(
	np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]).transpose(), # State
	np.array([
		[100.0,   0.0,   0.0, 0.0, 0.0],
		[0.0,   100.0,   0.0, 0.0, 0.0],
		[0.0,     0.0, 100.0, 0.0, 0.0],
		[0.0,     0.0,   0.0, 1.0, 0.0],
		[0.0,     0.0,   0.0, 0.0, 1.0]
	]).transpose(), # State Covariance
	np.array([
		[1.0, 0.0, 0.0,  dt, 0.0], # x'  = x + vx * dt
		[0.0, 1.0, 0.0, 0.0,  dt], # y'  = y + vy * dt
		[0.0, 0.0, 1.0, 0.0, 0.0], # t'  = t
		[0.0, 0.0, 0.0, 1.0, 0.0], # vx' = vx
		[0.0, 0.0, 0.0, 0.0, 1.0], # vy' = vy
	]), # State Transition Matrix
	np.array([
		[1.0, 0.0, 0.0, 0.0, 0.0], # x = gps_x
		[0.0, 1.0, 0.0, 0.0, 0.0], # y = gps_y
		[0.0, 0.0, 0.0, 0.0, 0.0], # no sensor readings for heading
		[0.0, 0.0, 0.0, 0.0, 0.0], # no sensor readings for velocity x
		[0.0, 0.0, 0.0, 0.0, 0.0], # no sensor readings for velocity y
	]), # Observation Matrix
	np.array([
		[0.0, 0.0, 0.0, 0.0, 0.0],
		[0.0, 0.0, 0.0, 0.0, 0.0],
		[0.0, 0.0, 0.0, 0.0, 0.0],
		[0.0, 0.0, 0.0, 0.0, 0.0],
		[0.0, 0.0, 0.0, 0.0, 0.0],
	]), # Process Noise Covariance
	np.array([
		[0.1 , 0.0, 0.0, 0.0, 0.0],
		[0.0,  0.1, 0.0, 0.0, 0.0],
		[0.0,  0.0, 1.0, 0.0, 0.0],
		[0.0,  0.0, 0.0, 1.0, 0.0],
		[0.0,  0.0, 0.0, 0.0, 1.0],
	]), # Observation Noise Covariance
)
		

last_time = times[1000]
idx = 1000
def do_stuff(i):
	global idx
	global last_time

	last_time = last_time + 100  # 0.1 seconds

	changed = False

	if idx < len(data):
		kalman_filter.prediction_step(np.zeros((5, 1)))

		while last_time > times[idx]:
			changed = True
			idx += 1

		if changed:
			sensors = np.array([[x_data[idx], y_data[idx], 0.0, 0.0, 0.0]]).transpose()
			kalman_filter.update_step(sensors)

		#colors = [(2.0 if i == idx else 1.0) for i in range(len(x_data))]
		colors = [('0.0' if i == idx else '0.5') for i in range(len(x_data))]
		sizes = [31.0 - float(f) * 30.0 for f in colors]

		ax.clear()
		ax.scatter(x_data, y_data, color=colors, s=sizes)

		#scatter.set(color=colors)
		#fig.canvas.draw()
		#fig.canvas.flush_events()

		print(f'got to {idx}')
		print(times[idx])
		print(x_data[idx], y_data[idx])

		p = (kalman_filter.state[0, 0], kalman_filter.state[1, 0])
		print(f'Position: {p}')

		v = (kalman_filter.state[3, 0], kalman_filter.state[4, 0])
		print(f'Velocity: {v}')

		c = tuple(kalman_filter.state_cov[i, i] for i in range(5))
		print(f'State Covariance: {c}')

		print()

ani = animation.FuncAnimation(fig, do_stuff, interval=100, cache_frame_data=False)

plt.show()