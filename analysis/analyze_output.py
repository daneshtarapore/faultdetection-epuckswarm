from collections import namedtuple

Dataline = namedtuple("Dataline", "time robot tolerators attackers")

def parse_votes(token):
	voters = []
	for subtoken in token.split(' ')[1:]:
		if subtoken == '-1' or subtoken == '':
			break
		voters.append(int(subtoken))
	return voters

def process_dataline(line):
	"""
	Processes a raw dataline into a dict of useful information
	"""

	if len(line.split('\t')) == 2:
		return None # there is no data here 

	
	tokens = line.split('\t')
	time = int(tokens[0].split(' ')[1])
	robot = int(tokens[1].split(' ')[1])
	tolerators = parse_votes(tokens[3])
	attackers = parse_votes(tokens[4])

	data = Dataline(time, robot, tolerators, attackers)

	print(data)
	return data

    


if __name__ == "__main__":
    
	assert process_dataline("1	0") is None
        
	process_dataline("Clock: 453	Id: 0	FV: 55	Consensus_Tolerators:  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1 	Consensus_Attackers:  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1 ")
	process_dataline('Clock: 1695	Id: 12	FV: 53	Consensus_Tolerators: 0 12 13 14 16 17 18 19 2 3 4 5 7 8 9  -1  -1  -1  -1  -1 	Consensus_Attackers: 1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1  -1 ')


    # # Read in the data from the nohup.txt file
    # step = 0
    # with open("../nohup.txt", 'r') as datafile:
    #     line = datafile.readline()
    #     processed = process_dataline(line)
        