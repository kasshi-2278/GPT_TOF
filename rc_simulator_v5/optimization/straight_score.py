"""Explicit extra reward authorized by the user; existing score is unchanged."""
import math
from optimization.time_score import add_time_score

RULES={'max_bonus':300,'max_abs_steer_deg':2,'max_yaw_rate_deg_s':6,
       'min_speed_cm_s':5,'min_segment_s':1,'min_segment_cm':20,
       'bonus':'300 * qualified straight distance / total travel distance; only on three-lap completion; stopped distance never credited.'}

def straight_metrics(rows):
    previous=None;distance=straight=duration=segment=0.0
    steering_variation=0.0;qualified=False
    def finish_run():
        nonlocal straight,duration,segment,qualified
        if not qualified and duration>=RULES['min_segment_s']-1e-7 and segment>=RULES['min_segment_cm']:
            straight+=segment
        duration=segment=0.0;qualified=False
    for row in rows:
        cur={k:float(row[k]) for k in ('time_s','x_cm','y_cm','heading_deg','speed_cm_s','steer_deg')}
        if previous is not None:
            dt=cur['time_s']-previous['time_s']
            if dt<=0:raise ValueError('Trace times must increase')
            travelled=math.hypot(cur['x_cm']-previous['x_cm'],cur['y_cm']-previous['y_cm'])
            distance+=travelled
            steering_variation+=abs(cur['steer_deg']-previous['steer_deg'])
            yaw=abs((cur['heading_deg']-previous['heading_deg']+180)%360-180)/dt
            eligible=(min(cur['speed_cm_s'],previous['speed_cm_s'])>=RULES['min_speed_cm_s'] and
                      max(abs(cur['steer_deg']),abs(previous['steer_deg']))<=RULES['max_abs_steer_deg'] and
                      yaw<=RULES['max_yaw_rate_deg_s'])
            if eligible:
                duration+=dt;segment+=travelled
                if qualified:straight+=travelled
                elif duration>=RULES['min_segment_s']-1e-7 and segment>=RULES['min_segment_cm']:
                    straight+=segment;qualified=True
            else:finish_run()
        previous=cur
    finish_run()
    return dict(travel_distance_cm=distance,straight_distance_cm=straight,
                straight_fraction=straight/distance if distance>0 else 0,
                steering_total_variation_deg=steering_variation)

def add_straight_score(result,rows):
    result=dict(result);result.update(straight_metrics(rows))
    bonus=RULES['max_bonus']*result['straight_fraction'] if result['completed_three_laps'] else 0
    result['original_score']=result['score'];result['straight_bonus']=round(bonus,6)
    result['smooth_score']=round(result['score']+bonus,6)
    return add_time_score(result)
