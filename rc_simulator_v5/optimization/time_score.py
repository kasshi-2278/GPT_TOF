"""Extra lap-time bonus requested by the user; previous scores are retained."""
import math

RULES={'reference_three_lap_seconds':600,'points_per_second':1,
       'formula':'max(0, 600 - three-lap total seconds); one point per second faster; only a complete, collision-free run with no danger event, sharp steering event, unnecessary stop, or program error.'}
DISTANCE_RULES={'reference_distance_cm':10500,'best_distance_cm':9500,'max_bonus':100,
                'formula':'100 * clamp((10500 cm - completed three-lap travel distance) / 1000 cm, 0, 1); only a clean completion (no collision, danger event, sharp steering event, unnecessary stop, or program error). Shorter measured path scores higher until the 9500 cm cap.'}

def time_bonus(completed,total_time_s,collisions=0,program_error=None,*,danger=0,sharp_steer=0,stops=0):
    if not completed or collisions or danger or sharp_steer or stops or program_error:return 0.0
    if not isinstance(total_time_s,(int,float)) or not math.isfinite(total_time_s) or total_time_s<=0:
        raise ValueError('Completed lap time must be positive and finite')
    return max(0.0,(RULES['reference_three_lap_seconds']-total_time_s)*RULES['points_per_second'])

def add_time_score(result):
    result=dict(result)
    bonus=time_bonus(result['completed_three_laps'],result.get('total_time_s'),
                     result.get('collisions',0),result.get('program_error'),
                     danger=result.get('danger',0),sharp_steer=result.get('sharp_steer',0),
                     stops=result.get('stops',0))
    result['time_bonus']=round(bonus,6)
    result['lap_speed_bonus']=round(bonus,6)
    total_time=result.get('total_time_s')
    result['average_lap_time_s']=round(total_time/3,6) if result.get('completed_three_laps') and total_time else None
    distance=result.get('travel_distance_cm')
    result['lap_rate_laps_per_minute']=round(180/total_time,6) if result.get('completed_three_laps') and total_time else None
    distance_bonus=0.0
    if (result.get('completed_three_laps') and not result.get('collisions',0) and
        not result.get('danger',0) and not result.get('sharp_steer',0) and not result.get('stops',0) and
        not result.get('program_error') and isinstance(distance,(int,float)) and
        math.isfinite(distance) and distance>=0):
        span=DISTANCE_RULES['reference_distance_cm']-DISTANCE_RULES['best_distance_cm']
        distance_bonus=DISTANCE_RULES['max_bonus']*max(0.0,min(1.0,(DISTANCE_RULES['reference_distance_cm']-distance)/span))
    result['distance_efficiency_bonus']=round(distance_bonus,6)
    result['race_score']=round(result['smooth_score']+bonus+distance_bonus,6)
    return result
