from collections import namedtuple
import numpy as np
import sys
import matplotlib.pyplot as plt

Dataline = namedtuple("Dataline", "time robot tolerators attackers")

def parse_votes(token):
	voters = []
	for subtoken in token.split(' ')[1:]:
		if subtoken == '-1' or subtoken == '' or subtoken == "\n":
			break
		voters.append(int(subtoken))
	return voters

def process_dataline(line):
	"""
	Processes a raw dataline into a dict of useful information
	"""

	if len(line.split('\t')) != 5:
		return None # there is no data here 

	
	tokens = line.split('\t')
	time = int(tokens[0].split(' ')[1])
	robot = int(tokens[1].split(' ')[1])
	tolerators = parse_votes(tokens[3])
	attackers = parse_votes(tokens[4])

	data = Dataline(time, robot, tolerators, attackers)

	return data


def process_file(file):
	"""
	Returns a list of processed datalines for the given file
	"""

	# Read in the data from the nohup.txt file
	lines = []
	with open(file, 'r') as datafile:
		for line in datafile.readlines():
			processed = process_dataline(line)
			if processed is not None:
				lines.append(processed)
	return lines


def time_sus(data: list[Dataline], num_robots):
	"""
	Returns the proportion of time the robot is considered faultly for each robot

	Robot is considered faulty if a majority of other robots considers it faulty, and back to normal when no one thinks so
	"""
	total_faulty_time = np.zeros(num_robots, dtype=int)
	currently_faulty = np.zeros(num_robots, dtype=int)
	declared_faulty_time = np.zeros(num_robots, dtype=int)

	max_time = data[-1].time
	min_time = data[0].time

	for line in data:
		is_comm_timestep = int(str(line.time)[-2:]) > 90

		if not is_comm_timestep:
			continue

		robot_ind = line.robot
		sus = len(line.attackers) >= num_robots / 2
		antisus = len(line.tolerators) >= num_robots / 2

		# Case 1: Neighbors now think you are faultly
		if sus and not currently_faulty[robot_ind]:
			currently_faulty[robot_ind] = 1
			declared_faulty_time[robot_ind] = line.time
			if line.robot == 15:
				print(f"Robot 15 declared fault at {line.time}")

		# Case 2: Neighbors no longer think you are faulty
		elif antisus and currently_faulty[robot_ind]:
			currently_faulty[robot_ind] = 0
			total_faulty_time[robot_ind] = total_faulty_time[robot_ind] + (line.time - declared_faulty_time[robot_ind])
			declared_faulty_time[robot_ind] = 0
			if line.robot == 15:
				print(f"Robot 15 declared safe at {line.time}")

	for robot, faulty in enumerate(list(currently_faulty)):
		if faulty:
			total_faulty_time[robot] = total_faulty_time[robot] + (max_time - declared_faulty_time[robot])




	return total_faulty_time / (max_time - min_time)







if __name__ == "__main__":
    
	assert process_dataline("1	0") is None
        
	process_dataline("Clock: 453	Id: 0	FV: 55	Consensus_Tolerators:  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1 	Consensus_Attackers:  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1 ")
	process_dataline('Clock: 1695	Id: 12	FV: 53	Consensus_Tolerators: 0 12 13 14 16 17 18 19 2 3 4 5 7 8 9  -1  -1  -1  -1  -1 	Consensus_Attackers: 1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1 ')

	data = process_file('original_data/SWARM_FORAGING/FAULT_ACTUATOR_LWHEEL_SETZERO/nohup_111' if len(sys.argv) < 2 else sys.argv[1])
	time_faulty = time_sus(data, 20)
	print(time_faulty)

	tf15 = []
	for i in range(20):
		exp = str(i+1)
		data = process_file('original_data/SWARM_FORAGING/FAULT_ACTUATOR_LWHEEL_SETZERO/nohup_' + exp*3)
		time_faulty = time_sus(data, 20)
		tf15.append(time_faulty[15])
	
	plt.boxplot(tf15)
	plt.ylim(0, 1)
	plt.show()


        