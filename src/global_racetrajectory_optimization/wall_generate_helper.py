# herlper file to create coordinates for walls in a costmap. You do not have to think about this

x = 1000
points = []
for y in range(400, 600):
    points.append([x,y])
print(points)