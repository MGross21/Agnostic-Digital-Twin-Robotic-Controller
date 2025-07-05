import json
import matplotlib.pyplot as plt

# Load the JSON data
with open('cup_tracker.json', 'r') as f:
    data = json.load(f)

# Assume data is a list of lists: [x, y, t]
x = [item[0] for item in data]
y = [item[1] for item in data]
z = [item[2] for item in data]

plt.figure(figsize=(8, 6))
plt.plot(x, y, marker='o')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Cup Tracking')
plt.grid(True)
plt.show()