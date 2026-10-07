"""Extra lap-time bonus requested by the user; previous scores are retained."""
import math

RULES={'reference_three_lap_seconds':600,'points_per_second':1,
       'formula':'max(0, 600 - three-lap total seconds); only collision-free completion without program error.'}

def time_bonus(completed,total_time_s,collisions=0,program_error=None):
    if not completed or collisions or program_error:return 0.0
    if not isinstance(total_time_s,(int,float)) or not math.isfinite(total_time_s) or total_time_s<=0:
        raise ValueError('Completed lap time must be positive and finite')
    return max(0.0,(RULES['reference_three_lap_seconds']-total_time_s)*RULES['points_per_second'])

def add_time_score(result):
    result=dict(result)
    bonus=time_bonus(result['completed_three_laps'],result.get('total_time_s'),
                     result.get('collisions',0),result.get('program_error'))
    result['time_bonus']=round(bonus,6)
    result['race_score']=round(result['smooth_score']+bonus,6)
    return result
