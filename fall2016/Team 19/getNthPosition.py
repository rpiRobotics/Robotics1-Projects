

## Function takes in the position of the first block
## in the jenga tower and the number of the block that
## is currently being placed and returns the position
## of that block and if it is rotated or not relative
## to the first block.  Assumes that the AR tags are
## placed in the center of the jenga pieces
def getNthPosition(x_in, y_in, z_in, n):
	## Dimensions of jenga block
        ## Need to be changed
	LENGTH = 3.0 # Will probably be 1/3 the width
	WIDTH = 1.0
	HEIGHT = 1.5
	
	## Output coordinates
	x = 0
	y = 0
	z = 0
	rotation = 0

	## Height increases by one unit of height
	## every third block
	z = HEIGHT * (n/3)

	## Rotation changes every third block
	rotation = (n/3) % 2

	## First 3 blocks only x changes
	if n % 6 < 3:
		x = WIDTH * (n%3)

	## Second set of 3 blocks. x is in the middle
	## of the tower. ((n%3)-1) = -1,0,1 depending
	## on n, so it will a third of the length down,
	## up, or not at all
	else:
		x = WIDTH
		y = ((n%3)-1) * (LENGTH/3.0)

	## Return values plus the initital coordinate
	return [x+x_in, y+y_in, z+z_in, rotation]


if __name__ == "__main__":
	for i in range(10):
		print getNthPosition(0,0,0,i)
