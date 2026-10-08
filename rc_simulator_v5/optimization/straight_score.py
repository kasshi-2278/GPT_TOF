"""Explicit extra reward authorized by the user; existing score is unchanged."""
import math
from optimization.time_score import add_time_score

RULES={'max_bonus':300,'max_abs_steer_deg':2,'max_yaw_rate_deg_s':6,
       'min_speed_cm_s':5,'min_segment_s':1,'min_segment_cm':20,
       'bonus':'300 * qualified straight distance / total travel distance; only on three-lap completion; stopped distance never credited.'}

CORNER_RULES={'max_bonus':100,'min_speed_cm_s':8,'min_yaw_rate_deg_s':4,
              'rate_full_quality_deg_s':12,'rate_zero_quality_deg_s':60,
              'jerk_full_quality_deg_s2':80,'jerk_zero_quality_deg_s2':600,
              'bonus':'100 * distance-weighted corner smoothness; only on three-lap completion without collision or program error. A corner sample requires speed >=8 cm/s and yaw rate >=4 deg/s. Quality combines steering-rate and steering-jerk scores linearly.'}

SLOWDOWN_RULES={'penalty_points_per_cm_s_speed_drop':0.02,'min_sample_drop_cm_s':0.3,
                'emergency_front_cm':60,'emergency_diagonal_cm':18,'emergency_side_cm':12,
                'sample_interval_s':0.05,
                'penalty':'0.02 points per cm/s speed drop above 0.3 cm/s per 0.05-second sample; no penalty inside the emergency-proximity envelope.'}

def straight_metrics(rows):
    previous=None;distance=straight=duration=segment=0.0
    steering_variation=0.0;qualified=False;slowdown=0.0;slowdown_events=0
    def finish_run():
        nonlocal straight,duration,segment,qualified
        if not qualified and duration>=RULES['min_segment_s']-1e-7 and segment>=RULES['min_segment_cm']:
            straight+=segment
        duration=segment=0.0;qualified=False
    for row in rows:
        cur={k:float(row[k]) for k in ('time_s','x_cm','y_cm','heading_deg','speed_cm_s','steer_deg')}
        for k in ('Fr','FrLh','RrLh','FrRh','RrRh'):
            if k in row:cur[k]=float(row[k])
        if previous is not None:
            dt=cur['time_s']-previous['time_s']
            if dt<=0:raise ValueError('Trace times must increase')
            travelled=math.hypot(cur['x_cm']-previous['x_cm'],cur['y_cm']-previous['y_cm'])
            distance+=travelled
            steering_variation+=abs(cur['steer_deg']-previous['steer_deg'])
            sensors=('Fr','FrLh','RrLh','FrRh','RrRh')
            if all(k in row and k in previous for k in sensors):
                readings=[float(row[k]) for k in sensors]
                emergency=(readings[0]<SLOWDOWN_RULES['emergency_front_cm'] or
                           min(readings[1],readings[3])<SLOWDOWN_RULES['emergency_diagonal_cm'] or
                           min(readings[2],readings[4])<SLOWDOWN_RULES['emergency_side_cm'])
                drop=previous['speed_cm_s']-cur['speed_cm_s']
                if not emergency and drop>SLOWDOWN_RULES['min_sample_drop_cm_s']:
                    slowdown+=drop-SLOWDOWN_RULES['min_sample_drop_cm_s'];slowdown_events+=1
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
                steering_total_variation_deg=steering_variation,
                avoidable_speed_drop_cm_s=slowdown,avoidable_slowdown_events=slowdown_events,
                slowdown_penalty=round(slowdown*SLOWDOWN_RULES['penalty_points_per_cm_s_speed_drop'],6))

def add_straight_score(result,rows):
    result=dict(result);rows=list(rows);result.update(straight_metrics(rows))
    bonus=RULES['max_bonus']*result['straight_fraction'] if result['completed_three_laps'] else 0
    result['original_score']=result['score'];result['straight_bonus']=round(bonus,6)
    result['smooth_score']=round(result['score']+bonus-result['slowdown_penalty'],6)
    result.update(corner_smoothness_metrics(rows))
    eligible=(result['completed_three_laps'] and result.get('collisions',0)==0 and
              not result.get('program_error'))
    corner_bonus=CORNER_RULES['max_bonus']*result['corner_smoothness_fraction'] if eligible else 0
    result['corner_smoothness_bonus']=round(corner_bonus,6)
    result['smooth_score']=round(result['smooth_score']+corner_bonus,6)
    return add_time_score(result)

def corner_smoothness_metrics(rows):
    """Distance-weight steering continuity only while the car is actually turning."""
    previous=None;corner_distance=weighted_quality=0.0
    turning_samples=0;rate_abs_sum=jerk_abs_sum=0.0
    for row in rows:
        cur={k:float(row[k]) for k in ('time_s','x_cm','y_cm','heading_deg','speed_cm_s','steer_deg')}
        if previous is not None:
            dt=cur['time_s']-previous['time_s']
            if dt<=0:raise ValueError('Trace times must increase')
            distance=math.hypot(cur['x_cm']-previous['x_cm'],cur['y_cm']-previous['y_cm'])
            yaw=abs((cur['heading_deg']-previous['heading_deg']+180)%360-180)/dt
            if min(cur['speed_cm_s'],previous['speed_cm_s'])>=CORNER_RULES['min_speed_cm_s'] and yaw>=CORNER_RULES['min_yaw_rate_deg_s'] and distance>0:
                steer_rate=(cur['steer_deg']-previous['steer_deg'])/dt
                prior_rate=previous.get('_steer_rate',steer_rate)
                jerk=abs(steer_rate-prior_rate)/dt
                rate_quality=clamp01((CORNER_RULES['rate_zero_quality_deg_s']-abs(steer_rate))/(CORNER_RULES['rate_zero_quality_deg_s']-CORNER_RULES['rate_full_quality_deg_s']))
                jerk_quality=clamp01((CORNER_RULES['jerk_zero_quality_deg_s2']-jerk)/(CORNER_RULES['jerk_zero_quality_deg_s2']-CORNER_RULES['jerk_full_quality_deg_s2']))
                quality=.5*(rate_quality+jerk_quality)
                corner_distance+=distance;weighted_quality+=distance*quality
                turning_samples+=1;rate_abs_sum+=abs(steer_rate);jerk_abs_sum+=jerk
            cur['_steer_rate']=(cur['steer_deg']-previous['steer_deg'])/dt
        previous=cur
    fraction=weighted_quality/corner_distance if corner_distance>0 else 0.0
    return dict(corner_distance_cm=round(corner_distance,6),corner_turning_samples=turning_samples,
                corner_smoothness_fraction=round(fraction,6),
                corner_mean_abs_steering_rate_deg_s=round(rate_abs_sum/turning_samples,6) if turning_samples else 0.0,
                corner_mean_abs_steering_jerk_deg_s2=round(jerk_abs_sum/turning_samples,6) if turning_samples else 0.0)

def clamp01(value):
    return max(0.0,min(1.0,value))
