import math

def find_largest_gap(game_state):
    # Set the number of sectors and search radius
    num_sectors = 12
    search_radius = 100
    ship_radius = 20

    # Get asteroid data from the game state
    asteroids_data = game_state["asteroids"]
    
    # Initialize a list to store weights for each sector
    sectors = [0] * num_sectors

    # Loop through each asteroid in the game state
    for asteroid in asteroids_data:
        # Get the position and size of the asteroid
        asteroid_position = asteroid["position"]
        asteroid_size = asteroid["size"]

        # Calculate the angle of the asteroid based on positive x-axis
        asteroid_angle = math.degrees(math.atan2(asteroid_position[1], asteroid_position[0]))

        # Adjust the angle to be positive (between 0 and 360)
        asteroid_angle = (asteroid_angle + 360) % 360

        # Divide the angle by 30 to determine the sector
        sector_index = int(asteroid_angle / (360 / num_sectors))

        # Update the sector value to 1
        sectors[sector_index] = 1

    # Find the largest gap in the sectors
    max_gap = 0
    current_gap = 0

    for i, sector in enumerate(sectors):
        if sector == 0:
            current_gap += 1
            if current_gap == 1:
                start_sector = i
        else:
            if current_gap > 0:
                end_sector = i
            current_gap = 0

        if current_gap > max_gap:
            max_gap = current_gap

    # Return the sector values and the size of the largest gap
    return sectors, max_gap, start_sector, end_sector

# Example usage:
game_state = {
    "asteroids": [
        {"position": (30, 40), "size": 1},
        {"position": (-20, 80), "size": 2},
        {"position": (30, 80), "size": 2},
        {"position": (-80, 20), "size": 3},
        {"position": (10, 80), "size": 4},
        {"position": (10, 80), "size": 4},
        {"position": (10, 80), "size": 4},
        {"position": (10, 80), "size": 4},
        {"position": (10, 80), "size": 4},
        {"position": (-70, 80), "size": 4},
        # {"position": (-70, -80), "size": 4},
        # {"position": (70, -80), "size": 4},
        # Add more asteroids as needed
    ]
}

result, largest_gap, start_sector, end_sector = find_largest_gap(game_state)
print("Sectors:", result)
print("Largest Gap:", largest_gap)
# print("Start Sector of Largest Gap:", start_sector)
print("End Sector of Largest Gap:", end_sector)


#* Assign end and start angle to every sector
#* Find the angle of every asteroid, based on positive x axis
#* Divide the angle by 30, and then assign it into one of the sectors 
#* Change the value of that sector to 1
#* Find the max zeroes