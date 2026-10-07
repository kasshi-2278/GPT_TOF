"""Minimal loadable function example: stops at a wall, does not navigate a lap."""
def decide(sensors, dt):
    # This file receives ONLY Fr/FrLh/RrLh/FrRh/RrRh in centimeters.
    if min(sensors['Fr'],sensors['FrLh'],sensors['FrRh']) < 60:
        return 0,0,'前方が近いので停止'
    return 25,0,'前進'
