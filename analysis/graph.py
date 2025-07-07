import csv
import matplotlib.pyplot as plt

# Open the CSV file and read it line by line
with open('@file:dynamic_pathfinding/large_batch.csv', 'r') as f:
    reader = csv.reader(f)

    # Extract the data from each row of the CSV file
    rows = [row for row in reader]

# Create a dictionary to store the number of successes by algorithm
success_counts = {}

# Loop through each row of the CSV file and increment the count of successes by algorithm
for row in rows:
    algo, num_walls, num_obstacles, success = row[0], int(row[1]), int(row[2]), row[3] == 'true'

    # Increment the count of successes by algorithm
    if algo not in success_counts:
        success_counts[algo] = 0
    success_counts[algo] += 1 if success else 0

# Create a plot comparing the number of successes by algorithm
plt.plot(success_counts, label='Success Count')
plt.xlabel('Algorithm')
plt.ylabel('Number of Successes')
plt.title('Comparison of Number of Successes by Algorithm')
plt.legend()
plt.show()
